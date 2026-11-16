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

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Instant;

use bowshock_model::{CfdGrid, CfdTable};
use bowshock_sim::from_model::Phase;
use iced::futures::channel::{mpsc, oneshot};
use iced::{keyboard, Element, Task};

use crate::app::{self, Ctx};
use crate::units::UnitSystem;
use crate::widgets::plot::{PlotMessage, PlotState};

pub mod cfd;
pub mod compute;
pub mod flutter;
pub mod import;
pub mod overlay;
mod plots;
pub mod readout;
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
    /// Solve the 3D stability table (Solve stability (3D)).
    SolveLift,
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
    /// An Analyze state showing Aerodynamics with the default conditions.
    pub fn new() -> Self {
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
            compare: compare_from_env(),
            compare_open: false,
            compare_out: None,
            wanted_cmp: None,
            compare_rev: 0,
            overlay: None,
            base_method: Method::Extended,
            cfd: Cfd::default(),
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

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Select(a) => {
                self.analysis = a;
                self.show_table = false;
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
            Message::Table(on) => self.show_table = on,
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
            Message::CfdCancel => {
                if let Some(s) = self.cfd.solving.take() {
                    s.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                    s.handle.abort();
                    self.cfd.failed = Some((s.key, "solve cancelled".to_string()));
                    ctx.notify("CFD solve cancelled");
                } else if let Some(s) = self.cfd.lift_solving.take() {
                    s.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                    s.handle.abort();
                    ctx.notify("3D stability solve cancelled");
                }
            }
            Message::SolveLift => return self.start_lift(),
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
        Task::perform(
            async move {
                let Some(file) = rfd::AsyncFileDialog::new().set_title("Export analysis").set_file_name(name).add_filter("CSV", &["csv"]).save_file().await
                else {
                    return Ok(None);
                };
                let path = file.path().to_path_buf();
                std::fs::write(&path, csv).map_err(|e| e.to_string())?;
                Ok(Some(path))
            },
            |result| app::Message::Analyze(Message::Exported(result)),
        )
    }

    /// Called by the app after every message while Analyze is showing: starts a computation when what is wanted changed and
    /// brings the plots and table up to date. The returned task carries the result back as [`Message::Done`].
    pub fn refresh(&mut self, ctx: &Ctx) -> Task<app::Message> {
        let phase = ctx.physics.phase();
        let solve = self.refresh_cfd(ctx, phase);
        let table = self.cfd_table_for(self.analysis);
        // The CFD table method with no table to draw (not solved yet, a geometry the solver cannot take) shows the method before it.
        let method = if self.method == Method::CfdTable && table.is_none() { self.base_method } else { self.method };
        let stamp = |t: &Option<Arc<CfdTable>>| t.as_ref().map(|t| (t.key, t.solved_at, t.lift.as_ref().map(|l| l.solved_at)));
        let key = Key {
            revision: ctx.doc.revision(),
            phase,
            analysis: self.analysis,
            method,
            conditions: self.cond,
            serial: self.serial,
            cfd: if method == Method::CfdTable { stamp(&table) } else { None },
        };
        let mut task = solve;
        if let Some(Comparison::Method(m)) = self.compare {
            let wants = uses_cfd(key.analysis);
            // Comparing with the CFD table needs one to compare with.
            let ready = m != Method::CfdTable || table.is_some();
            let ckey = Key { method: m, cfd: if m == Method::CfdTable { stamp(&table) } else { None }, ..key.clone() };
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
        self.sync_view(ctx);
        task
    }

    /// Solve stability (3D) was pressed: solves the lift table for the Mach range on screen, unless a solve already runs.
    fn start_lift(&mut self) -> Task<app::Message> {
        if self.method != Method::CfdTable || !uses_cfd(self.analysis) || self.cfd.solving.is_some() || self.cfd.lift_solving.is_some() {
            return Task::none();
        }
        let shown = matches!(self.cfd.status, cfd::Status::Fresh | cfd::Status::Stale) && self.cfd.table.is_some();
        let (Ok(plan), Some(nodes), true) = (self.cfd.plan.as_ref(), cfd::lift_nodes(self.cond.mach), shown) else {
            return Task::none();
        };
        self.cfd.serial += 1;
        let serial = self.cfd.serial;
        let cancel = Arc::new(AtomicBool::new(false));
        let key = Some(cfd::lift_key(plan.geometry, &nodes));
        let total = nodes.len();
        let (task, handle) = spawn_lift(cfd::LiftJob { plan: plan.clone(), nodes }, cancel.clone(), serial);
        self.cfd.lift_solving = Some(Solving { serial, done: 0, total, backend: None, key, cancel, handle });
        task
    }

    /// Re-solve (coarse) or Refine (medium) was pressed: the next refresh solves on `grid`.
    fn request_solve(&mut self, grid: CfdGrid) {
        self.serial += 1;
        self.cfd.force = (self.method == Method::CfdTable && uses_cfd(self.analysis) && self.cfd.solving.is_none() && self.cfd.lift_solving.is_none()).then_some(grid);
    }

    /// Keeps what the CFD table method knows about the document current (the mapped geometry, the stored table, whether it is
    /// stale) and starts a solve when the method is on and there is no table for what is on screen.
    fn refresh_cfd(&mut self, ctx: &Ctx, phase: Phase) -> Task<app::Message> {
        if !uses_cfd(self.analysis) {
            return Task::none();
        }
        let rocket = ctx.doc.rocket();
        if self.cfd.plan_at != Some((ctx.doc.revision(), phase, ctx.units)) {
            let plan = bowshock_sim::from_model::vehicle(rocket, ctx.physics.motors(), phase)
                .map_err(|e| e.to_string())
                .and_then(|v| cfd::plan(rocket, &v, ctx.units));
            self.cfd.plan = Arc::new(plan);
            self.cfd.plan_at = Some((ctx.doc.revision(), phase, ctx.units));
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
        let force = std::mem::take(&mut self.cfd.force);
        let (Ok(plan), Some(nodes)) = (self.cfd.plan.as_ref(), cfd::nodes(self.cond.mach)) else {
            return Task::none();
        };
        let key = Some(cfd::table_key(plan.geometry, self.cond.altitude, &nodes));
        if force.is_some() {
            self.cfd.failed = None;
        }
        let wanted = force.is_some() || (self.cfd.status == cfd::Status::Missing && self.cfd.failed.as_ref().map(|f| f.0) != Some(key));
        if self.method != Method::CfdTable || self.cfd.solving.is_some() || self.cfd.lift_solving.is_some() || !wanted {
            return Task::none();
        }
        self.cfd.serial += 1;
        let serial = self.cfd.serial;
        let cancel = Arc::new(AtomicBool::new(false));
        let grid = force.unwrap_or(CfdGrid::Coarse);
        let job = cfd::Job { plan: plan.clone(), altitude: self.cond.altitude, nodes: nodes.clone(), stage, grid };
        let (task, handle) = spawn_solve(job, cancel.clone(), serial);
        self.cfd.solving = Some(Solving { serial, done: 0, total: nodes.len(), backend: None, key, cancel, handle });
        task
    }

    /// The table the CFD method draws for `analysis`: the stored one, fresh or stale (a stale one is labelled so), if it covers
    /// what is plotted.
    fn cfd_table_for(&self, analysis: Analysis) -> Option<Arc<CfdTable>> {
        if !uses_cfd(analysis) || !matches!(self.cfd.status, cfd::Status::Fresh | cfd::Status::Stale) {
            return None;
        }
        let table = self.cfd.table.clone().filter(|t| analysis == Analysis::Stability || cfd::covers(t, self.cond.mach))?;
        // The 3D stability solve is drawn only while it is fresh; a stale one leaves Extended's margin, CP and CNα.
        let fresh = cfd::lift_status(&self.cfd.plan, table.lift.as_ref(), self.cond.mach) == cfd::Status::Fresh;
        Some(if table.lift.is_some() && !fresh { Arc::new(CfdTable { lift: None, ..(*table).clone() }) } else { table })
    }

    /// A message from the running solve: progress, then the table (stored with the document) or the reason there is none.
    fn cfd_event(&mut self, serial: u64, event: cfd::Event, ctx: &mut Ctx) {
        if let Some(lift) = self.cfd.lift_solving.as_mut().filter(|s| s.serial == serial) {
            match event {
                cfd::Event::LiftProgress { done, total, backend } => {
                    lift.done = done;
                    lift.total = total;
                    lift.backend = backend.or_else(|| lift.backend.take());
                }
                cfd::Event::LiftFinished(result) => {
                    self.cfd.lift_solving = None;
                    match result {
                        Ok(table) => {
                            let summary = format!("3D stability solved: {} points on {} in {}", table.points.len(), table.backend, duration_text(table.solve_seconds));
                            let stored = self.cfd.table.as_ref().map(|t| CfdTable { lift: Some(table), ..(**t).clone() });
                            if stored.is_some_and(|t| ctx.doc.store_cfd_table(t)) {
                                ctx.notify(summary);
                            } else {
                                ctx.notify("3D stability solve stopped: the solver returned no usable table");
                            }
                        }
                        Err(e) => ctx.notify(format!("3D stability solve stopped: {e}")),
                    }
                }
                _ => {}
            }
            return;
        }
        let Some(solving) = self.cfd.solving.as_mut().filter(|s| s.serial == serial) else {
            return;
        };
        match event {
            cfd::Event::LiftProgress { .. } | cfd::Event::LiftFinished(_) => {}
            cfd::Event::Progress { done, total, backend } => {
                solving.done = done;
                solving.total = total;
                solving.backend = backend.or_else(|| solving.backend.take());
            }
            cfd::Event::Finished(result) => {
                let key = solving.key;
                self.cfd.solving = None;
                match result {
                    Ok(table) => {
                        let summary =
                            format!("CFD table solved: {} Mach points on {} in {}", table.points.len(), table.backend, duration_text(table.solve_seconds));
                        if ctx.doc.store_cfd_table(table) {
                            ctx.notify(summary);
                        } else {
                            self.cfd.failed = Some((key, "the solver returned no usable table".to_string()));
                        }
                    }
                    Err(e) => {
                        ctx.notify(format!("CFD solve stopped: {e}"));
                        self.cfd.failed = Some((key, e));
                    }
                }
            }
        }
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
    /// The revision and phase `plan` was made for.
    plan_at: Option<(u64, Phase, UnitSystem)>,
    /// The geometry as the solver takes it, or why it cannot.
    pub(crate) plan: Arc<Result<cfd::Plan, String>>,
    /// How the stored table relates to the rocket and the conditions on screen.
    pub(crate) status: cfd::Status,
    /// The table stored with the document for the stack on show.
    pub(crate) table: Option<Arc<CfdTable>>,
    /// The solve in progress.
    pub(crate) solving: Option<Solving>,
    /// The 3D stability solve in progress.
    pub(crate) lift_solving: Option<Solving>,
    /// The key a solve was last wanted for and why it ended without a table (a failure or a cancel): no solve starts by itself
    /// again until the key changes or Re-solve is pressed.
    pub(crate) failed: Option<(Option<u64>, String)>,
    /// Re-solve or Refine was pressed: solve on this grid whether or not a table exists.
    force: Option<CfdGrid>,
    serial: u64,
}

impl Default for Cfd {
    fn default() -> Self {
        Cfd {
            plan_at: None,
            plan: Arc::new(Err("the geometry has not been mapped yet".to_string())),
            status: cfd::Status::Missing,
            table: None,
            solving: None,
            lift_solving: None,
            failed: None,
            force: None,
            serial: 0,
        }
    }
}

/// A CFD solve in progress.
pub(crate) struct Solving {
    serial: u64,
    /// Mach numbers solved.
    pub(crate) done: usize,
    /// Mach numbers to solve.
    pub(crate) total: usize,
    /// Where it runs, once the first point says so.
    pub(crate) backend: Option<String>,
    /// The key of the table it will produce.
    key: Option<u64>,
    cancel: Arc<AtomicBool>,
    handle: iced::task::Handle,
}

/// Whether the analysis uses the CFD table when the method is `CFD table` (the others do not use the method).
fn uses_cfd(analysis: Analysis) -> bool {
    matches!(analysis, Analysis::Aerodynamics | Analysis::Drag | Analysis::Stability)
}

/// Runs `job` on its own thread and delivers its progress and its end as [`Message::Cfd`]; the handle drops the stream (a
/// point in flight still finishes on the thread, then it sees `cancel` and stops).
fn spawn_solve(job: cfd::Job, cancel: Arc<AtomicBool>, serial: u64) -> (Task<app::Message>, iced::task::Handle) {
    let (tx, rx) = mpsc::unbounded();
    std::thread::spawn(move || {
        let result = cfd::solve(&job, &cancel, |event| {
            let _ = tx.unbounded_send(event);
        });
        let _ = tx.unbounded_send(cfd::Event::Finished(result));
    });
    Task::run(rx, move |event| app::Message::Analyze(Message::Cfd(serial, event))).abortable()
}

/// Runs the 3D stability solve of `job` on its own thread, like [`spawn_solve`].
fn spawn_lift(job: cfd::LiftJob, cancel: Arc<AtomicBool>, serial: u64) -> (Task<app::Message>, iced::task::Handle) {
    let (tx, rx) = mpsc::unbounded();
    std::thread::spawn(move || {
        let result = cfd::solve_lift(&job, &cancel, |event| {
            let _ = tx.unbounded_send(event);
        });
        let _ = tx.unbounded_send(cfd::Event::LiftFinished(result));
    });
    Task::run(rx, move |event| app::Message::Analyze(Message::Cfd(serial, event))).abortable()
}

/// `42 s`, `5 min 8 s`: how long a solve took.
fn duration_text(seconds: f64) -> String {
    let s = seconds.round() as u64;
    if s < 60 {
        format!("{s} s")
    } else {
        format!("{} min {} s", s / 60, s % 60)
    }
}

/// Opens the file dialog for an export of `source` and delivers its text as [`Message::CompareFile`].
fn pick_compare_file(source: import::Source) -> Task<app::Message> {
    Task::perform(
        async move {
            let Some(file) =
                rfd::AsyncFileDialog::new().set_title(format!("Import {} CSV", source.label())).add_filter("CSV", &["csv", "txt"]).pick_file().await
            else {
                return Ok(None);
            };
            let path = file.path().to_path_buf();
            let text = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
            let name = path.file_name().map_or_else(String::new, |n| n.to_string_lossy().into_owned());
            Ok(Some((source, name, text)))
        },
        |result| app::Message::Analyze(Message::CompareFile(result)),
    )
}

/// The file named by `BOWSHOCK_COMPARE_FILE`, as a comparison (a name with `rasaero` in it is read as RASAero II's format).
fn compare_from_env() -> Option<Comparison> {
    let path = std::env::var_os("BOWSHOCK_COMPARE_FILE")?;
    let name = std::path::Path::new(&path).file_name()?.to_string_lossy().into_owned();
    let source = if name.to_lowercase().contains("rasaero") { import::Source::RasAero } else { import::Source::OpenRocket };
    match std::fs::read_to_string(&path).map_err(|e| e.to_string()).and_then(|t| import::parse(&t, source)) {
        Ok(data) => Some(Comparison::File { data: Arc::new(data), name }),
        Err(e) => {
            eprintln!("BOWSHOCK_COMPARE_FILE: {e}");
            None
        }
    }
}

/// Runs `request` on its own thread and delivers the result as [`Message::Done`] (or [`Message::CompareDone`] when `compare`).
fn spawn(request: compute::Request, key: Key, compare: bool) -> Task<app::Message> {
    Task::perform(
        async move {
            let (tx, rx) = oneshot::channel();
            std::thread::spawn(move || {
                let _ = tx.send(Arc::new(compute::compute(&request)));
            });
            rx.await.ok()
        },
        move |out| {
            let key = key.clone();
            app::Message::Analyze(if compare { Message::CompareDone(key, out) } else { Message::Done(key, out) })
        },
    )
}
