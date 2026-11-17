//! Wire protocol between the client (the app) and the worker: one CBOR message per frame, a frame being a `u32` little-endian length
//! followed by the body. The stream starts with [`MAGIC`] from the worker, so output that a login shell prints before the worker
//! starts is skipped.

use bowshock_core::{AnalysisSpec, Error, LiftPolar, Polar, Progress};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::io::{self, Read, Write};

/// Version of the protocol; a worker of another version is refused.
pub const PROTOCOL: u32 = 1;
/// The worker's first bytes on stdout.
pub const MAGIC: &[u8; 8] = b"BSHKWRK1";
/// Largest accepted frame body.
pub const MAX_FRAME: usize = 256 << 20;

/// A solve job; one connection serves many.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Job {
    /// Zero-lift drag: [`Analysis::run`](bowshock_core::Analysis::run) over `machs`.
    Drag {
        /// The analysis to run.
        spec: AnalysisSpec,
        /// Mach numbers.
        machs: Vec<f64>,
    },
    /// Normal force and centre of pressure: [`Analysis::run_lift`](bowshock_core::Analysis::run_lift).
    Lift {
        /// The analysis to run.
        spec: AnalysisSpec,
        /// Mach numbers.
        machs: Vec<f64>,
        /// Angles of attack \[deg\].
        alphas_deg: Vec<f64>,
    },
}

/// Client to worker.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ClientMsg {
    /// First message: the protocol version and the worker version (source key) the client expects.
    Hello {
        /// [`PROTOCOL`] of the client.
        protocol: u32,
        /// Source key of the worker the client expects.
        key: String,
    },
    /// Run a job; `id` counts up per connection.
    Job {
        /// Job number.
        id: u64,
        /// The job.
        job: Job,
    },
    /// Stop the job `id` (ignored when it already ended).
    Cancel {
        /// Job number.
        id: u64,
    },
}

/// The device a worker runs on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeviceInfo {
    /// Name of the GPU the solver uses; `None` without a hardware GPU.
    pub gpu: Option<String>,
    /// Why there is no GPU (`gpu` is `None`).
    pub no_gpu_reason: Option<String>,
    /// Host name of the machine.
    pub host: String,
    /// Source key of the worker.
    pub worker_key: String,
}

/// Kind of a [`bowshock_core::Error`], as it crosses the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErrorKind {
    /// [`Error::InvalidInput`].
    InvalidInput,
    /// [`Error::Unsupported`].
    Unsupported,
    /// [`Error::Grid`].
    Grid,
    /// [`Error::Gpu`].
    Gpu,
    /// [`Error::Internal`].
    Internal,
}

/// Worker to client.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum WorkerMsg {
    /// Answer to `Hello`.
    Ready {
        /// [`PROTOCOL`] of the worker.
        protocol: u32,
        /// Source key of the worker.
        key: String,
        /// The device.
        device: DeviceInfo,
    },
    /// Progress of the running job.
    Progress(Progress),
    /// Result of a [`Job::Drag`].
    Drag(Polar),
    /// Result of a [`Job::Lift`].
    Lift(LiftPolar),
    /// The job failed.
    Error {
        /// Kind of the solver error.
        kind: ErrorKind,
        /// Its message.
        message: String,
    },
    /// The job ended because the client cancelled it.
    Cancelled,
}

impl WorkerMsg {
    /// The message that reports `e`.
    pub fn from_error(e: &Error) -> WorkerMsg {
        let (kind, message) = match e {
            Error::Cancelled => return WorkerMsg::Cancelled,
            Error::InvalidInput(m) => (ErrorKind::InvalidInput, m),
            Error::Unsupported(m) => (ErrorKind::Unsupported, m),
            Error::Grid(m) => (ErrorKind::Grid, m),
            Error::Gpu(m) => (ErrorKind::Gpu, m),
            Error::Internal(m) => (ErrorKind::Internal, m),
        };
        WorkerMsg::Error { kind, message: message.clone() }
    }
}

/// The solver error of an [`WorkerMsg::Error`].
pub fn to_error(kind: ErrorKind, message: String) -> Error {
    match kind {
        ErrorKind::InvalidInput => Error::InvalidInput(message),
        ErrorKind::Unsupported => Error::Unsupported(message),
        ErrorKind::Grid => Error::Grid(message),
        ErrorKind::Gpu => Error::Gpu(message),
        ErrorKind::Internal => Error::Internal(message),
    }
}

/// Writes one frame and flushes.
pub fn write_frame<W: Write, T: Serialize>(w: &mut W, msg: &T) -> io::Result<()> {
    let mut body = Vec::new();
    ciborium::into_writer(msg, &mut body).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
    if body.len() > MAX_FRAME {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "frame too large"));
    }
    w.write_all(&(body.len() as u32).to_le_bytes())?;
    w.write_all(&body)?;
    w.flush()
}

/// Reads one frame; `Ok(None)` at a clean end of the stream (before the length).
pub fn read_frame<R: Read, T: DeserializeOwned>(r: &mut R) -> io::Result<Option<T>> {
    let mut len = [0u8; 4];
    match r.read_exact(&mut len) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let n = u32::from_le_bytes(len) as usize;
    if n > MAX_FRAME {
        return Err(io::Error::new(io::ErrorKind::InvalidData, format!("frame of {n} bytes")));
    }
    let mut body = vec![0u8; n];
    r.read_exact(&mut body)?;
    ciborium::from_reader(body.as_slice()).map(Some).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))
}

/// Skips bytes until [`MAGIC`] has been read (at most 64 KiB of other output); `false` at the end of the stream.
pub fn sync_magic<R: Read>(r: &mut R) -> io::Result<bool> {
    let mut window = [0u8; 8];
    let mut filled = 0usize;
    let mut byte = [0u8; 1];
    for _ in 0..(64 << 10) {
        if r.read(&mut byte)? == 0 {
            return Ok(false);
        }
        if filled < 8 {
            window[filled] = byte[0];
            filled += 1;
        } else {
            window.rotate_left(1);
            window[7] = byte[0];
        }
        if filled == 8 && &window == MAGIC {
            return Ok(true);
        }
    }
    Err(io::Error::new(io::ErrorKind::InvalidData, "no worker handshake in the first 64 KiB of output"))
}
