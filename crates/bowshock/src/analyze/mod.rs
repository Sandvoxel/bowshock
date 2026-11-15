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
    built_for: Option<(u64, UnitSystem)>,
    pub(crate) built: Option<plots::Built>,
    pub(crate) table: Option<compute::Table>,
    pub(crate) show_table: bool,
    pub(crate) plot: PlotState,
    pub(crate) gauge: GaugeState,
    /// The result generation the gauge last jumped to (it springs between hovers, jumps to a new result).
    gauge_generation: u64,
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
        if self.wanted.as_ref() != Some(&key) {
            self.wanted = Some(key.clone());
            match compute::jobs(ctx.doc.rocket(), ctx.physics.motors(), key.phase) {
                Ok((jobs, omitted)) => {
                    self.error = None;
                    self.omitted = omitted;
                    self.pending = true;
                    task = spawn(compute::Request { analysis: key.analysis, method: key.method, conditions: key.conditions, jobs }, key);
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
        if self.built_for != Some((self.generation, ctx.units)) {
            let built = plots::build(&out, ctx.units);
            self.plot.set_full(built.group.x_extent());
            self.table = Some(out.table());
            self.built = Some(built);
            self.built_for = Some((self.generation, ctx.units));
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

/// Runs `request` on its own thread and delivers the result as [`Message::Done`] for `key`.
fn spawn(request: compute::Request, key: Key) -> Task<app::Message> {
    Task::perform(
        async move {
            let (tx, rx) = oneshot::channel();
            std::thread::spawn(move || {
                let _ = tx.send(Arc::new(compute::compute(&request)));
            });
            rx.await.ok()
        },
        move |out| app::Message::Analyze(Message::Done(key.clone(), out)),
    )
}
