//! The Analyze screen (design/components/AnalyzeScreen): analyses list, method and conditions on the left, linked plots in
//! the centre, values at the cursor on the right.
//!
//! The physics lives in [`compute`] (pure; runs on a worker thread so the UI never waits), the "values at cursor" in
//! [`readout`], the plot data in [`plots`] and the drawing in `view`. [`State::refresh`] is the one hook the app calls after
//! every message while Analyze is showing: it starts a new computation when the document, the analysis, the method or a
//! condition changed, and rebuilds the plots when a result arrived or the unit system changed.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`], [`State::on_key`], [`State::tick`] and [`State::animating`] with the shared [`Ctx`].
//! Plot zoom keys (`Enter`, `Esc`) are handled by the plots themselves while the pointer is over them.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use crate::app::worker;
use bowshock_core::Field3;
use bowshock_model::{CfdGrid, CfdTable};
use bowshock_sim::from_model::Phase;
use iced::{keyboard, Element, Task};

use crate::app::{self, Ctx};
use crate::units::UnitSystem;
use crate::widgets::plot::{PlotMessage, PlotState};

pub mod cfd;
pub mod compute;
mod flow_view;
pub mod flutter;
pub mod import;
pub mod optimize;
mod optimize_view;
pub mod overlay;
mod plots;
pub mod readout;
pub mod solve;
mod solve_demo;
mod solve_view;
mod view;

pub use compute::{Analysis, Conditions, FinishChoice, Method};

/// Which of the Mach fields a text edit belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachField {
    /// The start of the Mach range.
    Low,
    /// The end of the Mach range.
    High,
    /// The single Mach of the angle-of-attack sweep.
    Fixed,
}

/// The Analyze screen's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Pick an analysis in the left list.
    Select(Analysis),
    /// Pick the method.
    Method(Method),
    /// Run again (the physics is deterministic; this is mostly a way to see the timing).
    Resolve,
    /// Re-solve the table on the medium grid.
    Refine,
    /// A Mach field was typed in.
    MachText(MachField, String),
    /// Enter in a Mach field: parse and apply.
    MachCommit,
    /// Angle of attack of the Mach sweeps (rad).
    Alpha(f64),
    /// Largest angle of the angle sweep (rad).
    AlphaMax(f64),
    /// Altitude that sets the Reynolds number (m).
    Altitude(f64),
    /// Which CG the margin uses (`true` = burnout).
    Burnout(bool),
    /// The surface finish.
    Finish(FinishChoice),
    /// Monte Carlo runs.
    Runs(usize),
    /// Mean wind of the Monte Carlo (m/s).
    Wind(f64),
    /// From the plots (hover, zoom).
    Plot(PlotMessage),
    /// Nothing to do (messages of widgets that only display).
    Ignore,
    /// Switch between plots and the table view.
    Table(bool),
    /// Export the numbers as CSV (opens the save dialog).
    ExportCsv,
    /// The save dialog answered: the written path, `None` if cancelled, or the error.
    Exported(Result<Option<PathBuf>, String>),
    /// The worker finished the computation for this request.
    Done(Key, Option<Arc<compute::Output>>),
    /// Open or close the `Compare ▾` menu.
    CompareMenu,
    /// Compare with the other built-in method.
    CompareWith(Method),
    /// Pick a file of another tool to compare with (opens the dialog).
    ImportCompare(import::Source),
    /// The open dialog answered: the source, the file's name and its text; `None` if cancelled, or the error.
    CompareFile(Result<Option<(import::Source, String, String)>, String>),
    /// Drop the comparison.
    ClearCompare,
    /// The worker finished the comparison method's computation for this request.
    CompareDone(Key, Option<Arc<compute::Output>>),
    /// A message from the running CFD solve (with the solve's serial: a cancelled solve's last words are ignored).
    Cfd(u64, cfd::Event),
    /// Cancel the running CFD solve.
    CfdCancel,
    /// Keep the points a cancelled solve had solved as the table.
    CfdKeep,
    /// Drop the points a cancelled solve had solved.
    CfdDiscard,
    /// Include the 3D angles in the solve, or leave them out (a quick table of the drag alone).
    Angles(bool),
    /// Open Preferences at Compute (where the solves run).
    OpenPrefs,
    /// A message of the Optimize screen.
    Optimize(optimize_view::Msg),
    /// A message from the running optimisation (with the run's serial: a cancelled run's last words are ignored).
    OptEvent(u64, optimize::Event),
    /// A message of the flow viewer (the Flow field view of a table, the picture of a solve).
    Flow(flow_view::Msg),
}

/// What a computation was for; a result is used only while it is still the one wanted.
#[derive(Debug, Clone, PartialEq)]
pub struct Key {
    revision: u64,
    phase: Phase,
    analysis: Analysis,
    method: Method,
    conditions: Conditions,
    serial: u64,
    /// The CFD table the drag comes from (its key and when it was solved), for the CFD table method.
    cfd: Option<(u64, u64, Option<u64>)>,
    /// The key and solve time of each stage's table Dispersion flies on, for the CFD table method.
    stage_tables: Vec<(u64, u64)>,
}

/// The Analyze screen's state.
pub struct State {
    pub(crate) analysis: Analysis,
    pub(crate) method: Method,
    pub(crate) cond: Conditions,
    /// Text of the Mach fields while they are being edited.
    pub(crate) mach_text: [String; 3],
    serial: u64,
    wanted: Option<Key>,
    pub(crate) pending: bool,
    pub(crate) error: Option<String>,
    /// Configurations left out because only four colours exist.
    pub(crate) omitted: usize,
    /// The note for configurations that could not be built (`1 configuration skipped: …`).
    pub(crate) skipped_note: Option<String>,
    pub(crate) done: Option<Arc<compute::Output>>,
    pub(crate) computed_at: Option<Instant>,
    generation: u64,
    built_for: Option<(u64, UnitSystem, u64)>,
    pub(crate) built: Option<plots::Built>,
    pub(crate) table: Option<compute::Table>,
    pub(crate) show_table: bool,
    pub(crate) plot: PlotState,
    /// Where the crosshair rests when the pointer is not over the plots, in the plots' x units.
    default_x: Option<f64>,
    /// What the plots are compared with, if anything.
    pub(crate) compare: Option<Comparison>,
    /// Whether the `Compare ▾` menu is open.
    pub(crate) compare_open: bool,
    /// The other method's result while [`Comparison::Method`] is on.
    compare_out: Option<Arc<compute::Output>>,
    wanted_cmp: Option<Key>,
    /// Bumped whenever the comparison's data changes, so the plots are rebuilt.
    compare_rev: u64,
    /// The comparison's curves for the result on screen.
    pub(crate) overlay: Option<overlay::Overlay>,
    /// The method the CFD table falls back to while it has no table to draw: the last one chosen before it.
    pub(crate) base_method: Method,
    /// The CFD table method's state.
    pub(crate) cfd: Cfd,
    /// The optimiser's setup and run.
    pub(crate) optimize: optimize_view::State,
    /// The flow viewer: what its picture is coloured by, the camera, the Mach number of the Flow field view. Session-only.
    pub(crate) flow: flow_view::State,
}

impl Default for State {
    fn default() -> Self {
        State::new()
    }
}

fn mach_text(m: f64) -> String {
    format!("{m:.2}")
}

impl State {
    /// An Analyze state showing Aerodynamics with the default conditions, reading nothing from the host.
    pub fn new() -> Self {
        State::from_env(&crate::app::env::Env::for_tests())
    }

    /// [`State::new`] with the start-up comparison file and scripted CFD demos from `env`.
    pub fn from_env(env: &crate::app::env::Env) -> Self {
        let cond = Conditions::default();
        State {
            analysis: Analysis::Aerodynamics,
            method: Method::Extended,
            mach_text: [mach_text(cond.mach.0), mach_text(cond.mach.1), mach_text(cond.mach_fixed)],
            cond,
            serial: 0,
            wanted: None,
            pending: false,
            error: None,
            omitted: 0,
            skipped_note: None,
            done: None,
            computed_at: None,
            generation: 0,
            built_for: None,
            built: None,
            table: None,
            show_table: false,
            plot: PlotState::new((0.0, 1.0)),
            default_x: None,
            compare: env.compare_file.as_deref().and_then(compare_from_path),
            compare_open: false,
            compare_out: None,
            wanted_cmp: None,
            compare_rev: 0,
            overlay: None,
            base_method: Method::Extended,
            cfd: Cfd { demo: solve_demo::from_names(&env.cfd_demo), ..Cfd::default() },
            optimize: optimize_view::State::new(),
            flow: flow_view::State::new(),
        }
    }

    /// The result to show: the latest one for the analysis on screen.
    pub(crate) fn output(&self) -> Option<&Arc<compute::Output>> {
        self.done.as_ref().filter(|o| o.analysis == self.analysis)
    }

    /// Whether there is a CFD table to draw or compare with (fresh or stale).
    pub(crate) fn cfd_ready(&self) -> bool {
        self.cfd.table.is_some() && matches!(self.cfd.status, cfd::Status::Fresh | cfd::Status::Stale)
    }

    /// The one verdict on the stored table as a whole solve: the drag points and, when the solve takes them (Conditions: 3D angles), the
    /// 3D angles. Stale when either part was solved for other geometry or conditions, or the angles were left out of the solve and are
    /// wanted now.
    pub(crate) fn solve_status(&self) -> cfd::Status {
        let lift = cfd::lift_status(&self.cfd.plan, self.cfd.table.as_ref().and_then(|t| t.lift.as_ref()), self.cond.mach);
        cfd::solve_status(&self.cfd.status, &lift, self.cfd.angles)
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Select(a) => {
                self.analysis = a;
                self.show_table = false;
                self.flow.open = false;
                self.plot.set_hover(None);
            }
            Message::Method(m) => {
                self.method = m;
                if m != Method::CfdTable {
                    self.base_method = m;
                }
                if self.compare == Some(Comparison::Method(m)) {
                    self.compare = Some(Comparison::Method(other_method(m)));
                    self.compare_rev += 1;
                }
            }
            Message::Resolve => self.request_solve(CfdGrid::Coarse),
            Message::Refine => self.request_solve(CfdGrid::Medium),
            Message::MachText(field, text) => self.mach_text[field as usize] = text,
            Message::MachCommit => self.commit_mach(ctx),
            Message::Alpha(a) => self.cond.alpha = a.clamp(-30f64.to_radians(), 30f64.to_radians()),
            Message::AlphaMax(a) => self.cond.alpha_max = a.clamp(1f64.to_radians(), 45f64.to_radians()),
            Message::Altitude(h) => self.cond.altitude = h.clamp(0.0, 30_000.0),
            Message::Burnout(b) => self.cond.burnout = b,
            Message::Finish(f) => self.cond.finish = f,
            Message::Runs(n) => self.cond.runs = n,
            Message::Wind(w) => self.cond.wind = w.clamp(0.0, 25.0),
            Message::Plot(PlotMessage::Hover(None)) => self.plot.set_hover(self.default_x),
            Message::Plot(m) => self.plot.update(m),
            Message::Ignore => {}
            Message::Table(on) => {
                self.show_table = on;
                if on {
                    self.flow.open = false;
                }
            }
            Message::Flow(m) => {
                if matches!(m, flow_view::Msg::Open(true)) {
                    self.show_table = false;
                }
                self.flow.update(m);
            }
            Message::ExportCsv => return self.export(ctx),
            Message::Exported(result) => match result {
                Ok(Some(path)) => ctx.notify(format!("Wrote {}", path.display())),
                Ok(None) => {}
                Err(e) => ctx.notify(format!("Export failed: {e}")),
            },
            Message::Done(key, out) => {
                if self.wanted.as_ref() == Some(&key) {
                    self.pending = false;
                    match out {
                        Some(out) => {
                            self.done = Some(out);
                            self.generation += 1;
                            self.computed_at = Some(Instant::now());
                            self.error = None;
                        }
                        None => self.error = Some("the analysis stopped unexpectedly".to_string()),
                    }
                }
            }
            Message::CompareMenu => self.compare_open = !self.compare_open,
            Message::CompareWith(m) => {
                self.compare_open = false;
                self.compare = Some(Comparison::Method(m));
                self.compare_out = None;
                self.wanted_cmp = None;
                self.compare_rev += 1;
            }
            Message::ClearCompare => {
                self.compare_open = false;
                self.compare = None;
                self.compare_out = None;
                self.wanted_cmp = None;
                self.compare_rev += 1;
            }
            Message::ImportCompare(source) => {
                self.compare_open = false;
                return pick_compare_file(source);
            }
            Message::CompareFile(result) => match result {
                Ok(Some((source, name, text))) => match import::parse(&text, source) {
                    Ok(data) => {
                        ctx.notify(format!("Comparing with {} ({} rows from {name})", source.label(), data.rows.len()));
                        self.compare = Some(Comparison::File { data: Arc::new(data), name });
                        self.compare_out = None;
                        self.wanted_cmp = None;
                        self.compare_rev += 1;
                    }
                    Err(e) => ctx.notify(format!("Cannot read {name}: {e}")),
                },
                Ok(None) => {}
                Err(e) => ctx.notify(format!("Import failed: {e}")),
            },
            Message::CompareDone(key, out) => {
                if self.wanted_cmp.as_ref() == Some(&key) {
                    self.compare_out = out;
                    self.compare_rev += 1;
                }
            }
            Message::Cfd(serial, event) => self.cfd_event(serial, event, ctx),
            Message::CfdCancel => self.cancel_solve(ctx),
            Message::CfdKeep => self.keep_stopped(ctx),
            Message::CfdDiscard => self.cfd.stopped = None,
            Message::Angles(on) => self.set_angles(on),
            Message::OpenPrefs => return Task::done(app::Message::Prefs(crate::prefs::Message::Open(crate::prefs::Section::Compute))),
            Message::Optimize(m) => return self.optimize.update(m, ctx),
            Message::OptEvent(serial, event) => self.optimize.event(serial, event, ctx),
        }
        Task::none()
    }

    fn commit_mach(&mut self, ctx: &mut Ctx) {
        let parse = |t: &str| t.trim().parse::<f64>().ok().filter(|m| m.is_finite());
        let (lo, hi, fixed) = (parse(&self.mach_text[0]), parse(&self.mach_text[1]), parse(&self.mach_text[2]));
        let ok = |m: f64| (0.01..=8.0).contains(&m);
        if let (Some(lo), Some(hi)) = (lo, hi) {
            let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
            if ok(lo) && ok(hi) && hi - lo >= 0.05 {
                self.cond.mach = (lo, hi);
            } else {
                ctx.notify("Mach range: 0.01 to 8, at least 0.05 wide");
            }
        }
        if let Some(m) = fixed.filter(|m| ok(*m)) {
            self.cond.mach_fixed = m;
        }
        self.mach_text = [mach_text(self.cond.mach.0), mach_text(self.cond.mach.1), mach_text(self.cond.mach_fixed)];
    }

    fn export(&self, ctx: &Ctx) -> Task<app::Message> {
        let Some(out) = self.output() else {
            return Task::none();
        };
        let csv = out.table(ctx.units).csv();
        let name = format!("{}-{}.csv", ctx.doc.title(), out.analysis.name().to_lowercase().replace(' ', "-"));
        app::dialogs::save_file(ctx.env.export_file.clone(), "Export analysis", name, ("CSV", &["csv"]), move |path| {
            let res = match path {
                Some(p) => std::fs::write(&p, &csv).map(|()| Some(p)).map_err(|e| e.to_string()),
                None => Ok(None),
            };
            app::Message::Analyze(Message::Exported(res))
        })
    }

    /// Called by the app after every message while Analyze is showing: starts a computation when what is wanted changed and
    /// brings the plots and table up to date. The returned task carries the result back as [`Message::Done`].
    pub fn refresh(&mut self, ctx: &Ctx) -> Task<app::Message> {
        if self.analysis == Analysis::Optimize {
            // The optimiser owns its state and worker: nothing to compute for the plots.
            self.optimize.sync(ctx);
            return Task::none();
        }
        let phase = ctx.physics.phase();
        let solve = self.refresh_cfd(ctx, phase);
        // Dispersion flies each stage on its own stored table: a stage's table that is missing or stale flies on Extended.
        let stage_tables: Vec<Option<bowshock_sim::DragTable>> = if self.analysis == Analysis::Dispersion {
            (phase.first_stage()..=ctx.physics.stages().len().max(1)).map(|s| ctx.physics.drag_table(ctx.doc.rocket(), s).ok()).collect()
        } else {
            Vec::new()
        };
        let stage_stamp: Vec<(u64, u64)> = (phase.first_stage()..=ctx.physics.stages().len().max(1))
            .zip(&stage_tables)
            .filter_map(|(s, t)| t.as_ref().and_then(|_| ctx.doc.rocket().cfd_table(s as u32)).map(|t| (t.key, t.solved_at)))
            .collect();
        let table = self.cfd_table_for(self.analysis);
        // The CFD table method with no table to draw (not solved yet, a geometry the solver cannot take) shows the method before it.
        let no_table = table.is_none() || (self.analysis == Analysis::Dispersion && stage_tables.iter().all(Option::is_none));
        let method = if self.method == Method::CfdTable && no_table { self.base_method } else { self.method };
        let stamp = |t: &Option<Arc<CfdTable>>| t.as_ref().map(|t| (t.key, t.solved_at, t.lift.as_ref().map(|l| l.solved_at)));
        let key = Key {
            revision: ctx.doc.revision(),
            phase,
            analysis: self.analysis,
            method,
            conditions: self.cond,
            serial: self.serial,
            cfd: if method == Method::CfdTable { stamp(&table) } else { None },
            stage_tables: if method == Method::CfdTable { stage_stamp.clone() } else { Vec::new() },
        };
        let mut task = solve;
        if let Some(Comparison::Method(m)) = self.compare {
            let wants = uses_cfd(key.analysis);
            // Comparing with the CFD table needs one to compare with.
            let ready = m != Method::CfdTable || !no_table;
            let ckey = Key {
                method: m,
                cfd: if m == Method::CfdTable { stamp(&table) } else { None },
                stage_tables: if m == Method::CfdTable { stage_stamp.clone() } else { Vec::new() },
                ..key.clone()
            };
            if wants && ready && self.wanted_cmp.as_ref() != Some(&ckey) {
                self.wanted_cmp = Some(ckey.clone());
                if let Ok((jobs, _)) = compute::jobs(ctx.doc.rocket(), ctx.physics.motors(), ckey.phase) {
                    let request = compute::Request {
                        analysis: ckey.analysis,
                        method: m,
                        conditions: ckey.conditions,
                        jobs,
                        bands: ctx.doc.rocket().stability_bands(),
                        cfd: if m == Method::CfdTable { table.clone() } else { None },
                        drag_tables: if m == Method::CfdTable && ckey.analysis == Analysis::Dispersion { stage_tables.clone() } else { Vec::new() },
                    };
                    task = Task::batch([task, spawn(request, ckey, true)]);
                }
            } else if wants && !ready && self.wanted_cmp.take().is_some() {
                self.compare_out = None;
                self.compare_rev += 1;
            }
        }
        if self.wanted.as_ref() != Some(&key) {
            self.wanted = Some(key.clone());
            match compute::jobs(ctx.doc.rocket(), ctx.physics.motors(), key.phase) {
                Ok((jobs, left)) => {
                    self.error = None;
                    self.omitted = left.over_limit;
                    self.skipped_note = left.skipped_note();
                    self.pending = true;
                    let request = compute::Request {
                        analysis: key.analysis,
                        method: key.method,
                        conditions: key.conditions,
                        jobs,
                        bands: ctx.doc.rocket().stability_bands(),
                        cfd: if key.method == Method::CfdTable { table } else { None },
                        drag_tables: if key.method == Method::CfdTable && key.analysis == Analysis::Dispersion { stage_tables.clone() } else { Vec::new() },
                    };
                    task = Task::batch([task, spawn(request, key, false)]);
                }
                Err(e) => {
                    self.error = Some(e);
                    self.pending = false;
                    self.done = None;
                    self.built = None;
                    self.built_for = None;
                }
            }
        }
        self.sync_flow();
        self.sync_view(ctx);
        task
    }

    /// The angles option was changed. It is only what the next solve takes: the plots are not redone, but a table solved without the angles
    /// reads as stale (see [`State::solve_status`]) while they are wanted.
    fn set_angles(&mut self, on: bool) {
        self.cfd.angles = on;
    }

    /// Re-solve (coarse) or Refine (medium) was pressed: the next refresh solves on `grid`.
    fn request_solve(&mut self, grid: CfdGrid) {
        self.serial += 1;
        self.cfd.force = (self.method == Method::CfdTable && uses_cfd(self.analysis) && self.cfd.solving.is_none()).then_some(grid);
    }

    /// Keeps what the CFD table method knows about the document current (the mapped geometry, the stored table, whether it is
    /// stale) and starts a solve when the method is on and there is no table for what is on screen.
    fn refresh_cfd(&mut self, ctx: &Ctx, phase: Phase) -> Task<app::Message> {
        if !uses_cfd(self.analysis) {
            return Task::none();
        }
        let rocket = ctx.doc.rocket();
        if let Some(plan) = ctx.physics.cfd_plan(phase.first_stage()) {
            self.cfd.plan = Arc::new(plan.clone());
        }
        let stage = phase.first_stage() as u32;
        let stored = rocket.cfd_table(stage);
        self.cfd.status = cfd::status(&self.cfd.plan, stored, self.cond.altitude, self.cond.mach);
        match stored {
            Some(t)
                if self.cfd.table.as_ref().is_some_and(|a| {
                    (a.key, a.solved_at, a.lift.as_ref().map(|l| (l.key, l.solved_at))) == (t.key, t.solved_at, t.lift.as_ref().map(|l| (l.key, l.solved_at)))
                }) => {}
            Some(t) => self.cfd.table = Some(Arc::new(t.clone())),
            None => self.cfd.table = None,
        }
        // The fields belong to the table they were solved for and the geometry it was solved for.
        let geometry = (*self.cfd.plan).as_ref().ok().map(|p| p.geometry);
        self.cfd.fields.retain_for(self.cfd.table.as_deref(), geometry);
        let force = std::mem::take(&mut self.cfd.force);
        let (Ok(plan), Some(nodes)) = (self.cfd.plan.as_ref(), cfd::nodes(self.cond.mach)) else {
            return Task::none();
        };
        let key = Some(cfd::table_key(plan.geometry, self.cond.altitude, &nodes));
        if self.cfd.stopped.as_ref().is_some_and(|s| s.key != key) {
            self.cfd.stopped = None;
        }
        if force.is_some() {
            self.cfd.failed = None;
        }
        let wanted = force.is_some() || (self.cfd.status == cfd::Status::Missing && self.cfd.failed.as_ref().map(|f| f.0) != Some(key));
        if self.method != Method::CfdTable || self.cfd.solving.is_some() || !wanted {
            return Task::none();
        }
        self.cfd.serial += 1;
        let serial = self.cfd.serial;
        let grid = force.unwrap_or(CfdGrid::Coarse);
        let job = Arc::new(cfd::Job {
            plan: plan.clone(),
            altitude: self.cond.altitude,
            nodes: nodes.clone(),
            stage,
            grid,
            compute: ctx.compute.clone(),
            cpu_only: ctx.local_solver == crate::app::settings::LocalSolver::Cpu,
            stability: if self.cfd.angles { cfd::lift_nodes(self.cond.mach).unwrap_or_default() } else { Vec::new() },
        });
        let tracker = solve::Tracker::new(job.units());
        let script = self.cfd.demo.pop_front().map(|demo| solve_demo::events(demo, &job));
        let scripted = script.is_some();
        let (task, stream) = match script {
            Some(events) => spawn_script(events, serial),
            None => spawn_solve(job.clone(), serial),
        };
        self.cfd.stopped = None;
        self.cfd.summary = None;
        self.cfd.solving = Some(Solving {
            tracker,
            job: Some(job),
            grid,
            altitude: Some(self.cond.altitude.clamp(0.0, 20_000.0)),
            started: Instant::now(),
            cancelling: false,
            scripted,
            key,
            stream,
            picture: None,
            fields: Vec::new(),
        });
        task
    }

    /// The table the CFD method draws for `analysis`: the stored one, fresh or stale (a stale one is labelled so), if it covers
    /// what is plotted.
    fn cfd_table_for(&self, analysis: Analysis) -> Option<Arc<CfdTable>> {
        if !uses_cfd(analysis) || !matches!(self.cfd.status, cfd::Status::Fresh | cfd::Status::Stale) {
            return None;
        }
        let table = self.cfd.table.clone().filter(|t| matches!(analysis, Analysis::Stability | Analysis::Dispersion) || cfd::covers(t, self.cond.mach))?;
        // The 3D stability solve is drawn only while it is fresh; a stale one leaves Extended's margin, CP and CNα.
        let fresh = cfd::lift_status(&self.cfd.plan, table.lift.as_ref(), self.cond.mach) == cfd::Status::Fresh;
        Some(if table.lift.is_some() && !fresh { Arc::new(CfdTable { lift: None, ..(*table).clone() }) } else { table })
    }

    /// A message from the running solve: what it reports goes to the solve's tracker; its end stores the table (or the 3D solve), or
    /// says why there is none, or, after a cancel, offers the points solved.
    fn cfd_event(&mut self, serial: u64, event: cfd::Event, ctx: &mut Ctx) {
        let solving = self.cfd.solving.as_mut().filter(|s| s.stream.serial() == serial);
        let Some(solving) = solving else { return };
        match event {
            cfd::Event::Machines(infos) => solving.tracker.set_machines(&infos),
            cfd::Event::Machine { index, at, state } => solving.tracker.on_machine(index, at, state),
            cfd::Event::Iteration(it) => solving.tracker.on_iteration(&it),
            cfd::Event::Field(field) => solving.picture = Some(field.0),
            cfd::Event::PointField { field } => {
                solving.picture = Some(field.0.clone());
                solving.fields.push(field.0);
            }
            cfd::Event::Point(done) => solving.tracker.on_done(&done),
            cfd::Event::Tick => {}
            cfd::Event::Finished(result) => self.solve_finished(result, ctx),
        }
    }

    /// The solve ended: store its table (with its 3D angles, if it had them), or say why there is none, or (after a cancel) keep what
    /// was solved on offer.
    fn solve_finished(&mut self, result: Result<CfdTable, String>, ctx: &mut Ctx) {
        let Some(mut solving) = self.cfd.solving.take() else { return };
        solving.stream.abort();
        let key = solving.key;
        match result {
            Ok(table) => {
                solving.tracker.finish();
                let (summary, brief) = (solving.tracker.summary(table.solve_seconds), solving.tracker.brief(table.solve_seconds));
                let table_key = table.key;
                self.cfd.store_fields(&table, std::mem::take(&mut solving.fields));
                if ctx.doc.store_cfd_table(table) {
                    self.cfd.summary = Some((table_key, summary));
                    ctx.notify(brief);
                } else {
                    self.cfd.failed = Some((key, "the solver returned no usable table".to_string()));
                }
            }
            Err(e) if e == cfd::CANCELLED => {
                solving.tracker.halt();
                let seconds = solving.started.elapsed().as_secs_f64().max(solving.tracker.clock());
                self.cfd.failed = Some((key, "solve cancelled".to_string()));
                let how_far = solve_view::how_far_brief(&solving.tracker);
                match solving.job.as_deref().and_then(|job| cfd::partial_table(job, &solving.tracker, seconds)) {
                    Some(table) => {
                        ctx.notify(format!("Cancelled after {how_far}"));
                        let (grid, altitude) = (solving.grid, solving.altitude.unwrap_or_default());
                        let keep_label = cfd::keep_label(&table);
                        let fields = std::mem::take(&mut solving.fields);
                        self.cfd.stopped = Some(Stopped { key, tracker: solving.tracker, table: Some(table), seconds, grid, altitude, keep_label, fields });
                    }
                    None => ctx.notify("CFD solve cancelled"),
                }
            }
            Err(e) => {
                ctx.notify(format!("CFD solve stopped: {e}"));
                self.cfd.failed = Some((key, e));
            }
        }
    }

    /// Cancel was pressed: the solver ends the run in flight within a few tens of milliseconds and the solve ends with the points
    /// it had. A scripted solve (screenshots) ends at once.
    fn cancel_solve(&mut self, ctx: &mut Ctx) {
        let Some(s) = self.cfd.solving.as_mut() else { return };
        s.stream.request_cancel();
        s.cancelling = true;
        if s.scripted {
            let serial = s.stream.serial();
            self.cfd_event(serial, cfd::Event::Finished(Err(cfd::CANCELLED.to_string())), ctx);
        }
    }

    /// Keep N solved points was pressed: the table of the points a cancelled solve had is stored with the document.
    fn keep_stopped(&mut self, ctx: &mut Ctx) {
        let Some(stopped) = self.cfd.stopped.take() else { return };
        let Some(table) = stopped.table else { return };
        let toast = cfd::kept_text(&table);
        self.cfd.store_fields(&table, stopped.fields);
        if ctx.doc.store_cfd_table(table) {
            ctx.notify(toast);
        }
    }

    /// Keeps the flow viewer's picture current: the live snapshot of the solve running, else (in the Flow field view) the final field of
    /// the selected Mach number of the table on show. The viewer closes when there is no table to look at.
    fn sync_flow(&mut self) {
        let on = self.method == Method::CfdTable && uses_cfd(self.analysis);
        let table = self.cfd.table.clone().filter(|_| on && self.cfd_ready());
        let running = self.cfd.solving.as_ref();
        if table.is_none() || running.is_some() {
            self.flow.open = false;
        }
        // the Mach numbers of the table, and those the 3D stability solve has a field at
        self.flow.machs = table.as_ref().map(|t| t.points.iter().map(|p| p.mach).collect()).unwrap_or_default();
        if let Some(t) = &table {
            self.flow.machs.extend(self.cfd.fields.machs(t));
            self.flow.machs.sort_by(f64::total_cmp);
            self.flow.machs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        }
        if self.flow.selected.is_none_or(|m| !self.flow.machs.iter().any(|x| (x - m).abs() < 1e-9)) {
            self.flow.selected = flow_view::default_mach(&self.flow.machs);
        }
        // the angles of attack the stored result has at the selected Mach number, and the one on show
        self.flow.alphas = match (&table, self.flow.selected) {
            (Some(t), Some(m)) => self.cfd.fields.alphas_at(t, m),
            _ => Vec::new(),
        };
        if self.flow.selected_alpha.is_none_or(|a| !self.flow.alphas.iter().any(|x| (x - a).abs() < 1e-9)) {
            self.flow.selected_alpha = self.flow.alphas.first().copied();
        }
        let picture: Option<Arc<Field3>> = match (running, &table, self.flow.selected, self.flow.selected_alpha) {
            (Some(s), _, _, _) if on => s.picture.clone(),
            (None, Some(t), Some(m), Some(a)) if self.flow.open => self.cfd.fields.get(t, m, a).cloned(),
            _ => None,
        };
        self.flow.set_field(picture.as_ref());
    }

    fn sync_view(&mut self, ctx: &Ctx) {
        let Some(out) = self.done.clone() else {
            return;
        };
        if self.built_for != Some((self.generation, ctx.units, self.compare_rev)) {
            let active = ctx.doc.rocket().active_config_id();
            self.overlay = match &self.compare {
                Some(Comparison::File { data, .. }) => Some(data.overlay(out.conditions.alpha, out.conditions.mach_fixed)),
                Some(Comparison::Method(m)) => {
                    self.compare_out.as_ref().filter(|c| c.analysis == out.analysis).and_then(|c| overlay::Overlay::from_output(c, active, m.name()))
                }
                None => None,
            };
            let built = plots::build(&out, ctx.units, self.overlay.as_ref(), active);
            self.plot.set_full(built.group.x_extent());
            self.table = Some(out.table(ctx.units));
            self.built = Some(built);
            self.built_for = Some((self.generation, ctx.units, self.compare_rev));
        }
        let (Some(built), true) = (&self.built, out.analysis == self.analysis) else {
            return;
        };
        // The crosshair starts at the cursor's default (the flight's maximum Mach) and returns there when the pointer leaves.
        let active = ctx.doc.rocket().active_config_id();
        let previous = self.default_x;
        self.default_x = (out.analysis != Analysis::Dispersion).then(|| readout::default_cursor(&out, active) / built.scale.x_si);
        if self.plot.hover_x().is_none() || self.plot.hover_x() == previous {
            self.plot.set_hover(self.default_x);
        }
    }

    /// The shortcut `key` maps to on this screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, _key: &keyboard::Key, _modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        None
    }

    /// Advances animations (called on frame ticks while [`animating`](Self::animating)).
    pub fn tick(&mut self, _now: Instant) {}

    /// Whether the screen needs frame ticks.
    pub fn animating(&self) -> bool {
        false
    }

    /// The screen: the analyses, method and conditions; the linked plots; the values at the cursor.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        view::screen(self, ctx)
    }
}

/// What the plots are compared with.
#[derive(Debug, Clone, PartialEq)]
pub enum Comparison {
    /// The other built-in method.
    Method(Method),
    /// A file imported from OpenRocket or RASAero.
    File {
        /// The parsed rows.
        data: Arc<import::Imported>,
        /// The file's name.
        name: String,
    },
}

impl Comparison {
    /// The source name drawn on the series and in the menu.
    pub fn label(&self) -> String {
        match self {
            Comparison::Method(m) => m.name().to_string(),
            Comparison::File { data, .. } => data.source.label().to_string(),
        }
    }
}

/// The other built-in method a comparison starts with: Extended for Barrowman and the CFD table, Barrowman for Extended.
pub fn other_method(m: Method) -> Method {
    if m == Method::Extended {
        Method::Barrowman
    } else {
        Method::Extended
    }
}

/// The CFD table method's state: the document mapped for the solver, the table stored for it, and the solve in progress.
pub(crate) struct Cfd {
    /// The geometry as the solver takes it, or why it cannot.
    pub(crate) plan: Arc<Result<cfd::Plan, String>>,
    /// How the stored table relates to the rocket and the conditions on screen.
    pub(crate) status: cfd::Status,
    /// The table stored with the document for the stack on show.
    pub(crate) table: Option<Arc<CfdTable>>,
    /// The final flow fields of the points of that table, solved in this session: not saved with the document.
    pub(crate) fields: cfd::FieldStore,
    /// The solve in progress.
    pub(crate) solving: Option<Solving>,
    /// The solve takes the 3D angles too (Conditions: 3D angles); off, it is the drag table alone.
    pub(crate) angles: bool,
    /// A solve that was cancelled after some points, until its points are kept or discarded.
    pub(crate) stopped: Option<Stopped>,
    /// The account of the last solve that finished and the key of the table it made, shown while that table is the one on show.
    pub(crate) summary: Option<(u64, String)>,
    /// The key a solve was last wanted for and why it ended without a table (a failure or a cancel): no solve starts by itself
    /// again until the key changes or Re-solve is pressed.
    pub(crate) failed: Option<(Option<u64>, String)>,
    /// Re-solve or Refine was pressed: solve on this grid whether or not a table exists.
    force: Option<CfdGrid>,
    serial: u64,
    /// Scripted solves the next solves play instead of running the solver (`BOWSHOCK_CFD_DEMO`, for screenshots).
    demo: VecDeque<solve_demo::Demo>,
}

impl Cfd {
    /// Keeps the final fields of a solve with its table: those at zero incidence beside the table's drag points, those at an angle of
    /// attack beside the table's 3D part (only the Mach numbers it has).
    fn store_fields(&mut self, table: &CfdTable, fields: Vec<Arc<Field3>>) {
        let (angled, flat): (Vec<_>, Vec<_>) = fields.into_iter().partition(|f| f.alpha_deg.abs() > 1e-6);
        self.fields.replace(table, flat);
        if let Some(lift) = &table.lift {
            let kept = angled.into_iter().filter(|f| lift.points.iter().any(|p| (p.mach - f.mach).abs() < 1e-9));
            self.fields.replace_stability(table, kept);
        }
    }
}

impl Default for Cfd {
    fn default() -> Self {
        Cfd {
            plan: Arc::new(Err("the geometry has not been mapped yet".to_string())),
            status: cfd::Status::Missing,
            table: None,
            fields: cfd::FieldStore::default(),
            solving: None,
            angles: true,
            stopped: None,
            summary: None,
            failed: None,
            force: None,
            serial: 0,
            demo: VecDeque::new(),
        }
    }
}

/// A CFD solve in progress.
pub(crate) struct Solving {
    /// What the worker has reported.
    pub(crate) tracker: solve::Tracker,
    /// The job, for the table of a solve that is stopped early.
    job: Option<Arc<cfd::Job>>,
    /// The grid it solves on.
    pub(crate) grid: CfdGrid,
    /// The altitude that sets the Reynolds number (m).
    pub(crate) altitude: Option<f64>,
    /// When it began (the screen's clock).
    pub(crate) started: Instant,
    /// Cancel was pressed; the solver is ending the run in flight.
    pub(crate) cancelling: bool,
    /// The events are played from a script, not a solver: Cancel answers at once.
    scripted: bool,
    /// The key of the table it will produce.
    key: Option<u64>,
    stream: worker::StreamHandle,
    /// The latest picture of the flow of a point in flight (or the final field of the point that ended last): the live view.
    pub(crate) picture: Option<Arc<Field3>>,
    /// The final fields of the points that ended, for the store of the table the solve makes.
    fields: Vec<Arc<Field3>>,
}

/// A solve that was cancelled after some points: the partial curve and, with two or more drag points, the table to keep.
pub(crate) struct Stopped {
    /// The key of the table the solve was for.
    key: Option<u64>,
    /// What the solve had reported when it stopped.
    pub(crate) tracker: solve::Tracker,
    /// The table of the points solved (at least two), if they make one.
    pub(crate) table: Option<CfdTable>,
    /// How long it ran (s).
    pub(crate) seconds: f64,
    /// The grid it solved on.
    pub(crate) grid: CfdGrid,
    /// The altitude that set the Reynolds number (m).
    pub(crate) altitude: f64,
    /// The label of the Keep button: `Keep 5 solved points`.
    pub(crate) keep_label: String,
    /// The final fields of the points solved, kept with the table if it is.
    fields: Vec<Arc<Field3>>,
}

/// Whether the analysis uses the CFD table when the method is `CFD table` (the others do not use the method).
fn uses_cfd(analysis: Analysis) -> bool {
    matches!(analysis, Analysis::Aerodynamics | Analysis::Drag | Analysis::Stability | Analysis::Dispersion)
}

/// Seconds between the worker's beats.
const BEAT: std::time::Duration = std::time::Duration::from_millis(250);

/// Runs `job` on its own thread and delivers its progress and its end as [`Message::Cfd`], with a beat so the clock on screen moves.
fn spawn_solve(job: Arc<cfd::Job>, serial: u64) -> (Task<app::Message>, worker::StreamHandle) {
    worker::spawn_stream(
        serial,
        Some((BEAT, || cfd::Event::Tick)),
        move |cancel, tx| {
            cfd::Event::Finished(cfd::solve(&job, &cancel, move |event| {
                let _ = tx.unbounded_send(event);
            }))
        },
        |serial, event| app::Message::Analyze(Message::Cfd(serial, event)),
    )
}

/// Plays `events` as the messages of the solve `serial` instead of running the solver (see [`solve_demo`]).
fn spawn_script(events: Vec<cfd::Event>, serial: u64) -> (Task<app::Message>, worker::StreamHandle) {
    worker::spawn_iter(serial, events, |serial, event| app::Message::Analyze(Message::Cfd(serial, event)))
}

/// Opens the file dialog for an export of `source` and delivers its text as [`Message::CompareFile`].
fn pick_compare_file(source: import::Source) -> Task<app::Message> {
    app::dialogs::pick_named_text(format!("Import {} CSV", source.label()), ("CSV", &["csv", "txt"]), move |res| {
        app::Message::Analyze(Message::CompareFile(res.map(|opt| opt.map(|(name, text)| (source, name, text)))))
    })
}

/// The file at `path` (`BOWSHOCK_COMPARE_FILE`), as a comparison (a name with `rasaero` in it is read as RASAero II's format).
fn compare_from_path(path: &str) -> Option<Comparison> {
    let name = std::path::Path::new(path).file_name()?.to_string_lossy().into_owned();
    let source = if name.to_lowercase().contains("rasaero") { import::Source::RasAero } else { import::Source::OpenRocket };
    match std::fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|t| import::parse(&t, source)) {
        Ok(data) => Some(Comparison::File { data: Arc::new(data), name }),
        Err(e) => {
            eprintln!("BOWSHOCK_COMPARE_FILE: {e}");
            None
        }
    }
}

/// Runs `request` on its own thread and delivers the result as [`Message::Done`] (or [`Message::CompareDone`] when `compare`).
fn spawn(request: compute::Request, key: Key, compare: bool) -> Task<app::Message> {
    worker::perform(
        move || Arc::new(compute::compute(&request)),
        move |out| {
            let key = key.clone();
            app::Message::Analyze(if compare { Message::CompareDone(key, out) } else { Message::Done(key, out) })
        },
    )
}
