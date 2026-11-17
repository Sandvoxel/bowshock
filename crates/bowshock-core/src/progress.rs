//! Progress of a running solve: a callback that sees where each run is and can stop it.
//!
//! The solvers iterate for seconds to minutes per Mach number. [`Analysis::on_progress`](crate::Analysis::on_progress) registers a
//! callback that is called from the thread that runs the analysis, once per [`SAMPLE_INTERVAL`] solver iterations (the interval at
//! which the stop rules sample their monitor), with a [`Progress`] snapshot. Reporting reads numbers the solver already has: it does
//! not change the iteration, the stop rules or the result. The callback decides whether to go on: returning
//! [`ControlFlow::Break`] ends the run at once and the analysis returns [`Error::Cancelled`](crate::Error::Cancelled).
//!
//! The callback runs on the solver's thread between iterations: keep it short (hand the snapshot to a channel, throttle there).

use std::fmt;
use std::ops::ControlFlow;
use std::sync::Arc;

/// Solver iterations between two [`Progress`] reports: the sampling interval of the stop rules' monitor.
pub const SAMPLE_INTERVAL: usize = 50;

/// Where one run (one Mach number, one angle of attack) is.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Progress {
    /// Mach number of the run.
    pub mach: f64,
    /// Angle of attack \[rad\]: 0 for the zero-lift drag solver, the angle of the 3D stability solve.
    pub alpha: f64,
    /// Iterations done in this run.
    pub iteration: usize,
    /// The iteration cap of this run.
    pub max_iterations: usize,
    /// Orders of magnitude the density residual has dropped from its peak (0 until it has a peak).
    pub residual_drop_orders: f64,
    /// The quantity the stop rule watches at the latest sample: the total drag coefficient of a viscous drag run (on the analysis's
    /// reference area), the normal force coefficient of a 3D run; NaN before the first sample. Other runs report their own monitor
    /// (the nose drag of an Euler body with a sting, the base pressure of an Euler base wake) in the solver's internal units.
    pub monitor: f64,
    /// Range (max − min) of the monitor over the last stop-rule window relative to the magnitude of its mean; `None` until the
    /// window is full. A run stops on its stationary rule when this drops below [`Progress::tolerance`] (and the residual condition
    /// of the rule holds).
    pub spread: Option<f64>,
    /// The relative tolerance of the stationary rule, if the run has one.
    pub tolerance: Option<f64>,
}

/// The callback of [`Analysis::on_progress`](crate::Analysis::on_progress), cheap to clone.
#[derive(Clone)]
pub struct ProgressSink(Arc<dyn Fn(&Progress) -> ControlFlow<()> + Send + Sync>);

impl ProgressSink {
    /// A sink calling `f`.
    pub fn new(f: impl Fn(&Progress) -> ControlFlow<()> + Send + Sync + 'static) -> ProgressSink {
        ProgressSink(Arc::new(f))
    }

    /// Reports `progress`; `Break` asks the run to stop.
    pub(crate) fn report(&self, progress: &Progress) -> ControlFlow<()> {
        (self.0)(progress)
    }

    /// A sink that multiplies the monitor by `k` (the coefficient scaling of the reference area) before it reports.
    pub(crate) fn scaled(&self, k: f64) -> ProgressSink {
        let inner = self.clone();
        ProgressSink::new(move |p| inner.report(&Progress { monitor: p.monitor * k, ..*p }))
    }
}

impl fmt::Debug for ProgressSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ProgressSink")
    }
}

/// Range over the magnitude of the mean of the last `k` samples of `hist` (`(iteration, monitor)`); `None` with fewer than `k`
/// samples, `k = 0`, or a mean of zero.
pub(crate) fn spread(hist: &[(usize, f64)], k: usize) -> Option<f64> {
    if k == 0 || hist.len() < k {
        return None;
    }
    let w = &hist[hist.len() - k..];
    let (lo, hi, sum) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY, 0.0), |(l, h, s), c| (l.min(c.1), h.max(c.1), s + c.1));
    let mean = (sum / k as f64).abs();
    (mean > 0.0 && mean.is_finite()).then_some((hi - lo) / mean)
}

/// Orders of drop of a residual `r` below its reference `peak` (0 without a peak).
pub(crate) fn drop_orders(peak: f64, r: f64) -> f64 {
    if peak > 0.0 && r.is_finite() {
        (peak / r.max(1e-300)).log10()
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_spread_is_the_range_over_the_mean_of_the_last_window() {
        let hist: Vec<(usize, f64)> = [9.0, 1.0, 1.01, 0.99, 1.0].iter().enumerate().map(|(i, m)| ((i + 1) * 50, *m)).collect();
        // the last four samples: range 0.02, mean 1.0
        assert!((spread(&hist, 4).unwrap() - 0.02).abs() < 1e-12);
        // the early outlier is outside the window of 4 but inside a window of 5
        assert!(spread(&hist, 5).unwrap() > 1.0);
        assert_eq!(spread(&hist, 6), None, "the window is not full");
        assert_eq!(spread(&hist, 0), None);
        assert_eq!(spread(&[(50, 0.0), (100, 0.0)], 2), None, "a zero mean has no relative spread");
    }

    #[test]
    fn the_drop_is_in_orders_below_the_peak() {
        assert!((drop_orders(1.0, 1e-3) - 3.0).abs() < 1e-12);
        assert_eq!(drop_orders(0.0, 1e-3), 0.0);
        assert_eq!(drop_orders(1.0, f64::NAN), 0.0);
    }

    #[test]
    fn a_scaled_sink_scales_the_monitor_and_passes_the_verdict_through() {
        let seen = Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = seen.clone();
        let sink = ProgressSink::new(move |p| {
            log.lock().unwrap().push(p.monitor);
            if p.iteration >= 100 {
                ControlFlow::Break(())
            } else {
                ControlFlow::Continue(())
            }
        });
        let scaled = sink.scaled(2.0);
        let p =
            Progress { mach: 0.8, alpha: 0.0, iteration: 50, max_iterations: 1000, residual_drop_orders: 0.0, monitor: 0.25, spread: None, tolerance: None };
        assert_eq!(scaled.report(&p), ControlFlow::Continue(()));
        assert_eq!(scaled.report(&Progress { iteration: 100, ..p }), ControlFlow::Break(()));
        assert_eq!(*seen.lock().unwrap(), vec![0.5, 0.5]);
    }
}
