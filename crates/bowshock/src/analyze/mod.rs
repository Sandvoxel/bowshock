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
use std::sync::Arc;
use std::time::Instant;

use bowshock_sim::from_model::Phase;
use iced::futures::channel::oneshot;
use iced::{keyboard, Element, Task};

use crate::app::{self, Ctx};
use crate::units::UnitSystem;
use crate::widgets::gauge::GaugeState;
use crate::widgets::plot::{PlotMessage, PlotState};

pub mod compute;
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
    pub(crate) done: Option<Arc<compute::Output>>,
    pub(crate) computed_at: Option<Instant>,
    generation: u64,
    built_for: Option<(u64, UnitSystem, u64)>,
    pub(crate) built: Option<plots::Built>,
    pub(crate) table: Option<compute::Table>,
    pub(crate) show_table: bool,
    pub(crate) plot: PlotState,
    pub(crate) gauge: GaugeState,
    /// The result generation the gauge last jumped to (it springs between hovers, jumps to a new result).
    gauge_generation: u64,
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
            done: None,
            computed_at: None,
            generation: 0,
            built_for: None,
            built: None,
            table: None,
            show_table: false,
            plot: PlotState::new((0.0, 1.0)),
            gauge: GaugeState::new(0.0),
            gauge_generation: u64::MAX,
            compare: compare_from_env(),
            compare_open: false,
            compare_out: None,
            wanted_cmp: None,
            compare_rev: 0,
            overlay: None,
        }
    }

    /// The result to show: the latest one for the analysis on screen.
    pub(crate) fn output(&self) -> Option<&Arc<compute::Output>> {
        self.done.as_ref().filter(|o| o.analysis == self.analysis)
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Select(a) => {
                self.analysis = a;
                self.show_table = false;
            }
            Message::Method(m) => {
                if m == Method::CfdTable {
                    ctx.notify(compute::CFD_REASON);
                } else {
                    self.method = m;
                    if self.compare == Some(Comparison::Method(m)) {
                        self.compare = Some(Comparison::Method(other_method(m)));
                        self.compare_rev += 1;
                    }
                }
            }
            Message::Resolve => self.serial += 1,
            Message::MachText(field, text) => self.mach_text[field as usize] = text,
            Message::MachCommit => self.commit_mach(ctx),
            Message::Alpha(a) => self.cond.alpha = a.clamp(-30f64.to_radians(), 30f64.to_radians()),
            Message::AlphaMax(a) => self.cond.alpha_max = a.clamp(1f64.to_radians(), 45f64.to_radians()),
            Message::Altitude(h) => self.cond.altitude = h.clamp(0.0, 30_000.0),
            Message::Burnout(b) => self.cond.burnout = b,
            Message::Finish(f) => self.cond.finish = f,
            Message::Runs(n) => self.cond.runs = n,
            Message::Wind(w) => self.cond.wind = w.clamp(0.0, 25.0),
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
        let csv = out.table().csv();
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
    /// brings the plots, table and gauge up to date. The returned task carries the result back as [`Message::Done`].
    pub fn refresh(&mut self, ctx: &Ctx) -> Task<app::Message> {
        let key = Key {
            revision: ctx.doc.revision(),
            phase: ctx.physics.phase(),
            analysis: self.analysis,
            method: self.method,
            conditions: self.cond,
            serial: self.serial,
        };
        let mut task = Task::none();
        if let Some(Comparison::Method(m)) = self.compare {
            let ckey = Key { method: m, ..key.clone() };
            let wants = matches!(key.analysis, Analysis::Aerodynamics | Analysis::Drag | Analysis::Stability);
            if wants && self.wanted_cmp.as_ref() != Some(&ckey) {
                self.wanted_cmp = Some(ckey.clone());
                if let Ok((jobs, _)) = compute::jobs(ctx.doc.rocket(), ctx.physics.motors(), ckey.phase) {
                    let request =
                        compute::Request { analysis: ckey.analysis, method: m, conditions: ckey.conditions, jobs, bands: ctx.doc.rocket().stability_bands() };
                    task = spawn(request, ckey, true);
                }
            }
        }
        if self.wanted.as_ref() != Some(&key) {
            self.wanted = Some(key.clone());
            match compute::jobs(ctx.doc.rocket(), ctx.physics.motors(), key.phase) {
                Ok((jobs, omitted)) => {
                    self.error = None;
                    self.omitted = omitted;
                    self.pending = true;
                    let request = compute::Request {
                        analysis: key.analysis,
                        method: key.method,
                        conditions: key.conditions,
                        jobs,
                        bands: ctx.doc.rocket().stability_bands(),
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
            let built = plots::build(&out, ctx.units, self.overlay.as_ref());
            self.plot.set_full(built.group.x_extent());
            self.table = Some(out.table());
            self.built = Some(built);
            self.built_for = Some((self.generation, ctx.units, self.compare_rev));
        }
        let (Some(built), true) = (&self.built, out.analysis == self.analysis) else {
            return;
        };
        let active = ctx.doc.rocket().active_config_id();
        let hover = self.plot.hover_x().map(|x| built.scale.to_si(x));
        let margin = readout::margin_at(&out, hover, active).map(|m| m.0).or_else(|| ctx.physics.balance().map(|b| b.margin_cal));
        if let Some(margin) = margin {
            if self.gauge_generation == self.generation {
                self.gauge.set(margin as f32);
            } else {
                self.gauge.snap(margin as f32);
                self.gauge_generation = self.generation;
            }
        }
    }

    /// The shortcut `key` maps to on this screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, _key: &keyboard::Key, _modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        None
    }

    /// Advances animations (called on frame ticks while [`animating`](Self::animating)).
    pub fn tick(&mut self, now: Instant) {
        self.gauge.tick(now);
    }

    /// Whether the screen needs frame ticks.
    pub fn animating(&self) -> bool {
        self.gauge.moving()
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

/// The other built-in method.
pub fn other_method(m: Method) -> Method {
    if m == Method::Barrowman {
        Method::Extended
    } else {
        Method::Barrowman
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
