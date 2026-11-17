//! The client: [`Remote`] runs jobs on a worker that it reaches over ssh.

use crate::proto::{read_frame, sync_magic, to_error, write_frame, ClientMsg, DeviceInfo, Job, WorkerMsg, PROTOCOL};
use crate::provision::{self, SetupStatus, MISSING_MARKER};
use crate::WORKER_KEY;
use bowshock_core::{AnalysisSpec, Condition, Error, Field3, Polar, Progress};
use std::fmt;
use std::io::{Read, Write};
use std::ops::ControlFlow;
use std::process::Child;
use std::sync::{Arc, Mutex};

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
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    child: Option<Child>,
    stderr: Arc<Mutex<String>>,
    device: DeviceInfo,
    next_id: u64,
    broken: bool,
}

impl Remote {
    /// Connects, installing the worker on the remote first when it is missing (`status` reports the steps). Fails with
    /// [`RemoteError::NoGpu`] when the remote has no hardware GPU.
    pub fn connect(host: &RemoteHost, status: &dyn Fn(SetupStatus)) -> Result<Remote, RemoteError> {
        status(SetupStatus::Connecting);
        status(SetupStatus::Checking);
        let remote = match Remote::spawn(host, status) {
            Err(Missing::Missing) => {
                provision::install(host, status)?;
                match Remote::spawn(host, status) {
                    Err(Missing::Missing) => return Err(RemoteError::Setup("the worker is missing again right after it was installed".into())),
                    other => other.map_err(Missing::into_error)?,
                }
            }
            other => other.map_err(Missing::into_error)?,
        };
        if remote.device.gpu.is_none() {
            return Err(RemoteError::NoGpu(provision::no_gpu_message(host, &remote.device)));
        }
        status(SetupStatus::Ready(remote.device.clone()));
        Ok(remote)
    }

    /// Starts `bowshock-worker serve` over ssh and shakes hands.
    fn spawn(host: &RemoteHost, status: &dyn Fn(SetupStatus)) -> Result<Remote, Missing> {
        status(SetupStatus::Probing);
        let mut child = provision::ssh_spawn(host, &provision::serve_script(WORKER_KEY))?;
        let reader = Box::new(child.stdout.take().unwrap());
        let writer = Box::new(child.stdin.take().unwrap());
        let stderr = provision::drain(child.stderr.take().unwrap());
        match Remote::handshake(reader, writer, Some(child), stderr.clone(), WORKER_KEY, false) {
            Ok(r) => Ok(r),
            Err(e) => {
                // the stderr thread may still be catching up
                std::thread::sleep(std::time::Duration::from_millis(50));
                if stderr.lock().unwrap().contains(MISSING_MARKER) {
                    Err(Missing::Missing)
                } else {
                    Err(Missing::Error(e))
                }
            }
        }
    }

    /// A connection over the given streams to a worker that speaks the protocol (tests run the worker in a thread).
    /// `require_gpu` makes a worker without a GPU an error.
    #[doc(hidden)]
    pub fn over_streams(
        reader: impl Read + Send + 'static,
        writer: impl Write + Send + 'static,
        expect_key: &str,
        require_gpu: bool,
    ) -> Result<Remote, RemoteError> {
        Remote::handshake(Box::new(reader), Box::new(writer), None, Arc::new(Mutex::new(String::new())), expect_key, require_gpu)
    }

    fn handshake(
        mut reader: Box<dyn Read + Send>,
        mut writer: Box<dyn Write + Send>,
        child: Option<Child>,
        stderr: Arc<Mutex<String>>,
        key: &str,
        require_gpu: bool,
    ) -> Result<Remote, RemoteError> {
        let remote_stderr = || stderr.lock().unwrap().trim().to_string();
        let closed = |s: String| if s.is_empty() { RemoteError::Protocol("the worker closed the connection".into()) } else { RemoteError::Ssh(s) };
        match sync_magic(&mut reader) {
            Ok(true) => {}
            Ok(false) => {
                std::thread::sleep(std::time::Duration::from_millis(50));
                return Err(closed(remote_stderr()));
            }
            Err(e) => return Err(RemoteError::Protocol(e.to_string())),
        }
        write_frame(&mut writer, &ClientMsg::Hello { protocol: PROTOCOL, key: key.to_string() }).map_err(|e| RemoteError::Protocol(e.to_string()))?;
        let device = match read_frame::<_, WorkerMsg>(&mut reader) {
            Ok(Some(WorkerMsg::Ready { protocol, key: worker_key, device })) => {
                if protocol != PROTOCOL {
                    return Err(RemoteError::Protocol(format!("the worker speaks protocol {protocol}, this program {PROTOCOL}")));
                }
                if worker_key != key && key != "unavailable" {
                    return Err(RemoteError::Protocol(format!("the worker is version {worker_key}, this program expects {key}")));
                }
                device
            }
            Ok(Some(_)) => return Err(RemoteError::Protocol("the worker did not answer the Hello with Ready".into())),
            Ok(None) => return Err(closed(remote_stderr())),
            Err(e) => return Err(RemoteError::Protocol(e.to_string())),
        };
        if require_gpu && device.gpu.is_none() {
            return Err(RemoteError::NoGpu("the worker has no GPU".into()));
        }
        Ok(Remote { reader, writer, child, stderr, device, next_id: 1, broken: false })
    }

    /// The remote device.
    pub fn device(&self) -> &DeviceInfo {
        &self.device
    }

    /// Runs [`Analysis::run`](bowshock_core::Analysis::run) of `spec` over `conditions` on the remote. `progress` sees every progress
    /// report; `Break` cancels the job and the call returns `Solver(Cancelled)`. The final fields of the points (with
    /// [`AnalysisSpec::keep_fields`]) arrive in frames of their own and are in the result's `field3`; the live snapshots of a solve
    /// stay on the worker.
    pub fn solve(&mut self, spec: &AnalysisSpec, conditions: &[Condition], progress: &dyn Fn(&Progress) -> ControlFlow<()>) -> Result<Polar, RemoteError> {
        let mut fields: Vec<(usize, Field3)> = Vec::new();
        let result = self.run_job(Job::Solve { spec: spec.clone(), conditions: conditions.to_vec() }, progress, &mut fields)?;
        let WorkerMsg::Solve(mut polar) = result else {
            return Err(self.protocol("the worker answered a solve job with another result"));
        };
        for (point, field) in fields {
            match polar.points.get_mut(point) {
                Some(p) => p.field3 = Some(field),
                None => return Err(self.protocol("the worker sent the field of a point the result does not have")),
            }
        }
        Ok(polar)
    }

    fn protocol(&mut self, msg: &str) -> RemoteError {
        self.broken = true;
        RemoteError::Protocol(msg.into())
    }

    /// The error for a stream that ended or broke: ssh's message when it has one.
    fn lost(&mut self, detail: Option<String>) -> RemoteError {
        self.broken = true;
        std::thread::sleep(std::time::Duration::from_millis(50));
        let s = self.stderr.lock().unwrap().trim().to_string();
        if !s.is_empty() {
            RemoteError::Ssh(s)
        } else {
            RemoteError::Protocol(detail.unwrap_or_else(|| "the connection to the worker was lost".into()))
        }
    }

    fn run_job(&mut self, job: Job, progress: &dyn Fn(&Progress) -> ControlFlow<()>, fields: &mut Vec<(usize, Field3)>) -> Result<WorkerMsg, RemoteError> {
        if self.broken {
            return Err(RemoteError::Protocol("the connection is closed".into()));
        }
        let id = self.next_id;
        self.next_id += 1;
        if let Err(e) = write_frame(&mut self.writer, &ClientMsg::Job { id, job }) {
            return Err(self.lost(Some(e.to_string())));
        }
        let mut cancelled = false;
        loop {
            let msg = match read_frame::<_, WorkerMsg>(&mut self.reader) {
                Ok(Some(m)) => m,
                Ok(None) => return Err(self.lost(None)),
                Err(e) => return Err(self.lost(Some(e.to_string()))),
            };
            match msg {
                WorkerMsg::Progress(p) => {
                    if !cancelled && progress(&p).is_break() {
                        cancelled = true;
                        if let Err(e) = write_frame(&mut self.writer, &ClientMsg::Cancel { id }) {
                            return Err(self.lost(Some(e.to_string())));
                        }
                    }
                }
                WorkerMsg::Cancelled => return Err(RemoteError::Solver(Error::Cancelled)),
                WorkerMsg::Error { kind, message } => return Err(RemoteError::Solver(to_error(kind, message))),
                WorkerMsg::Ready { .. } => return Err(self.protocol("unexpected Ready from the worker")),
                WorkerMsg::PointField { point, field } => fields.push((point, *field)),
                result @ WorkerMsg::Solve(_) => {
                    return if cancelled { Err(RemoteError::Solver(Error::Cancelled)) } else { Ok(result) };
                }
            }
        }
    }
}

impl Drop for Remote {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            // closing stdin ends the worker; the kill covers a worker in the middle of a solve
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// `spawn`'s failure: a missing worker (to install) or a real error.
enum Missing {
    Missing,
    Error(RemoteError),
}

impl From<RemoteError> for Missing {
    fn from(e: RemoteError) -> Missing {
        Missing::Error(e)
    }
}

impl Missing {
    fn into_error(self) -> RemoteError {
        match self {
            Missing::Error(e) => e,
            Missing::Missing => RemoteError::Setup("the worker is not installed".into()),
        }
    }
}

/// Connects to `host` (installing the worker when needed) and returns the device, then hangs up.
pub fn test_connection(host: &RemoteHost, status: &dyn Fn(SetupStatus)) -> Result<DeviceInfo, RemoteError> {
    Remote::connect(host, status).map(|r| r.device.clone())
}
