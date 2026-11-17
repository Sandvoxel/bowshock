//! The client: [`Remote`] runs jobs on a worker that it reaches over ssh.

use crate::proto::DeviceInfo;
use crate::provision::SetupStatus;
use bowshock_core::{AnalysisSpec, LiftPolar, Polar, Progress};
use std::fmt;
use std::ops::ControlFlow;

/// An ssh destination.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RemoteHost {
    /// `user@host` or an alias of the user's ssh config.
    pub target: String,
    /// ssh port when it is not the default.
    pub port: Option<u16>,
}

/// Why a remote operation failed.
#[derive(Debug)]
pub enum RemoteError {
    /// ssh itself failed (unreachable, authentication, no key): with ssh's own message.
    Ssh(String),
    /// Installing the worker failed.
    Setup(String),
    /// The remote has no hardware GPU: names what is missing.
    NoGpu(String),
    /// The worker misbehaved (version mismatch, broken stream).
    Protocol(String),
    /// The solver failed ([`bowshock_core::Error::Cancelled`] after a cancel).
    Solver(bowshock_core::Error),
}

impl fmt::Display for RemoteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RemoteError::Ssh(m) | RemoteError::Setup(m) | RemoteError::NoGpu(m) | RemoteError::Protocol(m) => f.write_str(m),
            RemoteError::Solver(e) => e.fmt(f),
        }
    }
}

impl std::error::Error for RemoteError {}

/// A connection to a worker.
pub struct Remote {
    device: DeviceInfo,
}

impl Remote {
    /// Connects, installing the worker on the remote first when it is missing (`status` reports the steps).
    pub fn connect(_host: &RemoteHost, _status: &dyn Fn(SetupStatus)) -> Result<Remote, RemoteError> {
        Err(RemoteError::Setup("not implemented".into()))
    }

    /// The remote device.
    pub fn device(&self) -> &DeviceInfo {
        &self.device
    }

    /// Runs [`Analysis::run`](bowshock_core::Analysis::run) of `spec` on the remote. `progress` sees every progress report;
    /// `Break` cancels the job and the call returns `Solver(Cancelled)`.
    pub fn run_drag(&mut self, _spec: &AnalysisSpec, _machs: &[f64], _progress: &dyn Fn(&Progress) -> ControlFlow<()>) -> Result<Polar, RemoteError> {
        Err(RemoteError::Setup("not implemented".into()))
    }

    /// Runs [`Analysis::run_lift`](bowshock_core::Analysis::run_lift) of `spec` on the remote (see [`Remote::run_drag`]).
    pub fn run_lift(
        &mut self,
        _spec: &AnalysisSpec,
        _machs: &[f64],
        _alphas_deg: &[f64],
        _progress: &dyn Fn(&Progress) -> ControlFlow<()>,
    ) -> Result<LiftPolar, RemoteError> {
        Err(RemoteError::Setup("not implemented".into()))
    }
}

/// Connects to `host` (installing the worker when needed) and returns the device, then hangs up.
pub fn test_connection(host: &RemoteHost, status: &dyn Fn(SetupStatus)) -> Result<DeviceInfo, RemoteError> {
    Remote::connect(host, status).map(|r| r.device.clone())
}
