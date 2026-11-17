//! The solves on another computer's GPU: the app side of `bowshock-remote`.
//!
//! A solve opens one [`Session`] per remote machine (an SSH connection to a `bowshock-worker` it has provisioned if need be), runs
//! Mach numbers through it with the same events a local solve sends, and drops it when the solve ends. A connection or set-up
//! failure takes that machine out of the solve (see [`super::share`]); there is no quiet fall back to this computer's CPU.

use super::run::{Event, Outbox};
use crate::analyze::solve::MachineState;
use bowshock_core as solver;
use bowshock_remote::{DeviceInfo, Remote, RemoteError, RemoteHost, SetupStatus};
use std::ops::ControlFlow;

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

/// A connection to a remote GPU for the length of one solve.
pub(super) struct Session {
    remote: Remote,
    /// What the screen calls the machine.
    name: String,
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

/// Opens the session of machine `index`, called `name`, at `host`: sends [`Event::Machine`] for each step of the set-up and once
/// connected. Blocks (the first connection to a computer that has no worker builds one, minutes).
pub(super) fn connect(host: &RemoteHost, name: &str, index: usize, started: std::time::Instant, send: &Outbox) -> Result<Session, String> {
    let at = || started.elapsed().as_secs_f64();
    let status = |s: SetupStatus| send(Event::Machine { index, at: at(), state: MachineState::Connecting(status_text(&s)) });
    let remote = Remote::connect(host, &status).map_err(|e| error_text(name, &e))?;
    let label = device_label(remote.device());
    send(Event::Machine { index, at: at(), state: MachineState::Ready(label.clone()) });
    Ok(Session { remote, name: name.to_string(), label })
}

impl Session {
    fn failed(&self, e: RemoteError, mach: f64) -> Failed {
        match e {
            RemoteError::Solver(solver::Error::Cancelled) => Failed::Cancelled,
            RemoteError::Solver(e) => Failed::Message(format!("Mach {mach:.2}: {e}")),
            other => Failed::Message(error_text(&self.name, &other)),
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

/// Tests the connection to `host`, called `name` (provisioning the worker if need be): blocking; `report` gets each step.
pub fn test(host: &RemoteHost, name: &str, report: impl Fn(Probe)) -> Probe {
    Probe::Done(
        bowshock_remote::test_connection(host, &|s| report(Probe::Status(status_text(&s))))
            .map(|device| device_label(&device))
            .map_err(|e| error_text(name, &e)),
    )
}
