//! The report of the last background run (the Simulate button, F5, the button's menu): one line per flight launched, the
//! outcome each gets as it lands, and when the toast under the top bar may go. Mutated through `Ctx::{start_run,
//! finish_run_line}`; drawn by `chrome::run_toast`.

use std::time::{Duration, Instant};

use super::Ctx;

/// How long the finished report stays up.
const REPORT_TIME: Duration = Duration::from_secs(6);

/// The headline numbers of one finished flight, SI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunSummary {
    /// Peak altitude, m.
    pub apogee: f64,
    /// Maximum velocity, m/s.
    pub max_speed: f64,
    /// Time from liftoff to touchdown, s.
    pub flight_time: f64,
}

/// One flight of a run.
#[derive(Debug, Clone, PartialEq)]
pub struct RunLine {
    /// The flight's id on the Simulate screen.
    pub id: u64,
    /// The flight's name.
    pub name: String,
    /// `None` while it flies; else its summary, or why it could not fly.
    pub outcome: Option<Result<RunSummary, String>>,
}

/// The flights of the last run and how far they are.
#[derive(Debug, Clone, PartialEq)]
pub struct RunReport {
    /// One line per flight launched, in launch order.
    pub lines: Vec<RunLine>,
    /// Lines still flying.
    pub pending: usize,
    /// When the report goes away; set once nothing is pending.
    pub until: Option<Instant>,
}

impl RunReport {
    /// Lines that could not fly or failed.
    pub fn failed(&self) -> usize {
        self.lines.iter().filter(|l| matches!(l.outcome, Some(Err(_)))).count()
    }
}

impl Ctx {
    /// Replaces the report with a new run of `lines`.
    pub fn start_run(&mut self, lines: Vec<RunLine>) {
        let pending = lines.iter().filter(|l| l.outcome.is_none()).count();
        let until = (pending == 0).then(|| Instant::now() + REPORT_TIME);
        self.run = Some(RunReport { lines, pending, until });
    }

    /// Records the outcome of flight `id` in the report, if it is on it and still flying.
    pub fn finish_run_line(&mut self, id: u64, outcome: Result<RunSummary, String>) {
        let Some(report) = self.run.as_mut() else { return };
        let Some(line) = report.lines.iter_mut().find(|l| l.id == id && l.outcome.is_none()) else { return };
        line.outcome = Some(outcome);
        report.pending = report.lines.iter().filter(|l| l.outcome.is_none()).count();
        if report.pending == 0 {
            report.until = Some(Instant::now() + REPORT_TIME);
        }
    }

    /// Whether a run's report is up and the clock must keep ticking for it.
    pub(super) fn run_animating(&self) -> bool {
        self.run.as_ref().is_some_and(|r| r.pending > 0 || r.until.is_some())
    }

    /// Whether any flight of the report is still flying.
    pub fn run_pending(&self) -> bool {
        self.run.as_ref().is_some_and(|r| r.pending > 0)
    }
}
