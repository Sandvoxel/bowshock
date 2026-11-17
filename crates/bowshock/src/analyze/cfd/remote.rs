//! The solves on another computer's GPU (`Compute::Remote`): the app side of `bowshock-remote`.
//!
//! A solve opens one [`Session`] (an SSH connection to a `bowshock-worker` it has provisioned if need be), runs each Mach number
//! through it with the same events a local solve sends, and drops it when the solve ends. A connection or set-up failure is the
//! solve's error: there is no quiet fall back to this computer's CPU.

use super::run::{Event, Outbox};
use crate::app::settings::Compute;
use bowshock_core as solver;
use bowshock_remote::{DeviceInfo, Remote, RemoteError, RemoteHost, SetupStatus};
use std::ops::ControlFlow;

/// Where a solve runs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Route {
    /// This computer.
    Local,
    /// The GPU behind this SSH host.
    Remote(RemoteHost),
    /// Remote compute with no target typed in.
    NoTarget,
}

/// Where a solve under `compute` runs.
pub fn route(compute: &Compute) -> Route {
    match (compute.is_remote(), compute.host()) {
        (false, _) => Route::Local,
        (true, Some(host)) => Route::Remote(host),
        (true, None) => Route::NoTarget,
    }
}

/// The text of the error of remote compute with no target.
pub const NO_TARGET: &str = "Remote GPU: type the target (user@host) in the Compute control, or switch to This computer.";

/// How a run through a [`Session`] failed.
pub(super) enum Failed {
    /// The cancel flag ended it.
    Cancelled,
    /// Anything else, as text for the screen.
    Message(String),
}

impl Failed {
    /// A failure of a local run at `mach`.
    pub(super) fn local(mach: f64, e: solver::Error) -> Failed {
        match e {
            solver::Error::Cancelled => Failed::Cancelled,
            e => Failed::Message(format!("Mach {mach:.2}: {e}")),
        }
    }
}

/// A connection to the remote GPU for the length of one solve.
pub(super) struct Session {
    remote: Remote,
    /// `NVIDIA GeForce RTX 3090 on omarchy`.
    pub label: String,
}

/// `<gpu> on <host>`.
pub fn device_label(device: &DeviceInfo) -> String {
    format!("{} on {}", device.gpu.as_deref().unwrap_or("no GPU"), device.host)
}

/// The set-up step as a line of text for the screen.
pub fn status_text(status: &SetupStatus) -> String {
    match status {
        SetupStatus::Connecting => "Connecting".to_string(),
        SetupStatus::Checking => "Checking the remote worker".to_string(),
        SetupStatus::Uploading { bytes } => format!("Uploading the worker ({:.1} MB)", *bytes as f64 / 1e6),
        SetupStatus::Building { line } => {
            let line = line.trim();
            if line.is_empty() {
                "Building the worker on the remote".to_string()
            } else {
                format!("Building the worker: {line}")
            }
        }
        SetupStatus::Probing => "Looking for the GPU".to_string(),
        SetupStatus::Ready(device) => format!("Ready: {}", device_label(device)),
    }
}

/// An error of the remote as one line for the screen, with `target` named.
pub fn error_text(target: &str, error: &RemoteError) -> String {
    let what = match error {
        RemoteError::Ssh(e) => format!("could not connect over SSH: {}", e.trim()),
        RemoteError::Setup(e) => format!("could not set up the worker: {}", e.trim()),
        RemoteError::NoGpu(e) => format!("no usable GPU: {}", e.trim()),
        RemoteError::Protocol(e) => format!("the worker misbehaved: {}", e.trim()),
        RemoteError::Solver(e) => e.to_string(),
    };
    format!("Remote GPU {target}: {what}")
}

/// Opens the session of a solve under `compute`: `None` for this computer. Sends [`Event::Setup`] for each step of the set-up
/// and [`Event::Device`] once connected. Blocks (the first connection to a computer that has no worker builds one, minutes).
pub(super) fn open(compute: &Compute, send: &Outbox) -> Result<Option<Session>, String> {
    let host = match route(compute) {
        Route::Local => return Ok(None),
        Route::NoTarget => return Err(NO_TARGET.to_string()),
        Route::Remote(host) => host,
    };
    let status = |s: SetupStatus| send(Event::Setup(status_text(&s)));
    let remote = Remote::connect(&host, &status).map_err(|e| error_text(&host.target, &e))?;
    let label = device_label(remote.device());
    send(Event::Device(label.clone()));
    Ok(Some(Session { remote, label }))
}

impl Session {
    fn failed(&self, e: RemoteError, mach: f64) -> Failed {
        match e {
            RemoteError::Solver(solver::Error::Cancelled) => Failed::Cancelled,
            RemoteError::Solver(e) => Failed::Message(format!("Mach {mach:.2}: {e}")),
            other => Failed::Message(error_text(&self.label, &other)),
        }
    }

    /// The zero-lift drag at `mach` of the analysis `spec` describes.
    pub fn drag(&mut self, spec: &solver::AnalysisSpec, mach: f64, progress: &dyn Fn(&solver::Progress) -> ControlFlow<()>) -> Result<solver::Polar, Failed> {
        self.remote.run_drag(spec, &[mach], progress).map_err(|e| self.failed(e, mach))
    }

    /// The 3D lift points at `mach` and `alphas_deg`.
    pub fn lift(
        &mut self,
        spec: &solver::AnalysisSpec,
        mach: f64,
        alphas_deg: &[f64],
        progress: &dyn Fn(&solver::Progress) -> ControlFlow<()>,
    ) -> Result<solver::LiftPolar, Failed> {
        self.remote.run_lift(spec, &[mach], alphas_deg, progress).map_err(|e| self.failed(e, mach))
    }
}

/// What a connection test reports.
#[derive(Debug, Clone)]
pub enum Probe {
    /// A set-up step.
    Status(String),
    /// The end: `NVIDIA GeForce RTX 3090 on omarchy`, or why not.
    Done(Result<String, String>),
}

/// Tests the connection to `compute`'s host (provisioning the worker if need be): blocking; `report` gets each step.
pub fn test(compute: &Compute, report: impl Fn(Probe)) -> Probe {
    let host = match route(compute) {
        Route::Remote(host) => host,
        Route::NoTarget => return Probe::Done(Err(NO_TARGET.to_string())),
        Route::Local => return Probe::Done(Err("Compute is set to This computer.".to_string())),
    };
    Probe::Done(
        bowshock_remote::test_connection(&host, &|s| report(Probe::Status(status_text(&s))))
            .map(|device| device_label(&device))
            .map_err(|e| error_text(&host.target, &e)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_route_follows_the_setting() {
        assert_eq!(route(&Compute::Local), Route::Local);
        assert_eq!(route(&Compute::remote("me@box:2200")), Route::Remote(RemoteHost { target: "me@box".to_string(), port: Some(2200) }));
        assert_eq!(route(&Compute::remote("  ")), Route::NoTarget);
    }

    #[test]
    fn remote_with_no_target_is_the_solves_error() {
        let send: Outbox = std::sync::Arc::new(|_| {});
        assert_eq!(open(&Compute::remote(""), &send).err().as_deref(), Some(NO_TARGET));
        assert!(matches!(open(&Compute::Local, &send), Ok(None)));
    }

    #[test]
    fn a_host_that_does_not_exist_fails_the_solve_with_ssh_s_words() {
        use super::super::{solve, test_plan, Job, CANCELLED};
        use std::sync::atomic::AtomicBool;
        use std::sync::{Arc, Mutex};
        let plan = test_plan();
        let compute = Compute::remote("nobody@invalid.invalid");
        let job = Job { plan, altitude: 0.0, nodes: vec![0.6, 2.0], stage: 1, grid: bowshock_model::CfdGrid::Coarse, compute, cpu_only: false };
        let events = Arc::new(Mutex::new(Vec::new()));
        let seen = events.clone();
        let err = solve(&job, &Arc::new(AtomicBool::new(false)), move |e| seen.lock().unwrap().push(e)).expect_err("nothing answers there");
        assert!(err.starts_with("Remote GPU nobody@invalid.invalid: "), "{err}");
        assert_ne!(err, CANCELLED);
        assert!(!events.lock().unwrap().iter().any(|e| matches!(e, Event::Device(_) | Event::Point(_))), "no solve ran, on any device");
    }
}
