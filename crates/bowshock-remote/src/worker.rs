//! The worker side: the serve loop of `bowshock-worker serve` and the device report of `bowshock-worker probe`.

use crate::proto::{read_frame, write_frame, ClientMsg, DeviceInfo, Job, WorkerMsg, MAGIC, PROTOCOL};
use crate::WORKER_KEY;
use bowshock_core::{Analysis, Error};
use std::io::{self, Read, Write};
use std::ops::ControlFlow;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Progress frames are sent at most this often (the first one of a job always).
const PROGRESS_INTERVAL: Duration = Duration::from_millis(100);

/// The device this process runs on.
pub fn device_info() -> DeviceInfo {
    let gpu = bowshock_core::gpu_name();
    let no_gpu_reason = gpu.is_none().then(no_gpu_reason);
    DeviceInfo { gpu, no_gpu_reason, host: host_name(), worker_key: WORKER_KEY.to_string() }
}

fn host_name() -> String {
    if let Ok(out) = std::process::Command::new("hostname").output() {
        let name = String::from_utf8_lossy(&out.stdout).trim().to_string();
        if !name.is_empty() {
            return name;
        }
    }
    std::env::var("HOSTNAME").or_else(|_| std::env::var("COMPUTERNAME")).unwrap_or_else(|_| "unknown".into())
}

/// Why no hardware GPU was found, as far as the file system tells.
fn no_gpu_reason() -> String {
    if cfg!(target_os = "linux") {
        let has_icd = ["/usr/share/vulkan/icd.d", "/etc/vulkan/icd.d", "/usr/local/share/vulkan/icd.d"]
            .iter()
            .any(|d| std::fs::read_dir(Path::new(d)).is_ok_and(|mut e| e.next().is_some()));
        if !has_icd {
            return "no Vulkan driver files (ICD) are installed".into();
        }
        return "Vulkan reports only a software device or no adapter: the vendor driver or the Vulkan loader is missing or broken".into();
    }
    "no hardware GPU adapter was found".into()
}

/// `bowshock-worker probe`: the device as one JSON line.
pub fn probe_json() -> String {
    serde_json::to_string(&device_info()).expect("DeviceInfo serialises")
}

/// Serves jobs on stdin/stdout until the client hangs up.
pub fn serve_stdio() -> io::Result<()> {
    serve(io::stdin(), io::stdout(), |a| a)
}

/// Serves the jobs that arrive on `reader` and answers on `writer` until `reader` ends. `prepare` edits every analysis before it
/// runs (tests shrink the 3D grid with it; the binary passes the identity).
pub fn serve<R, W>(reader: R, writer: W, prepare: impl Fn(Analysis) -> Analysis) -> io::Result<()>
where
    R: Read + Send + 'static,
    W: Write + Send + 'static,
{
    let writer = Arc::new(Mutex::new(writer));
    {
        let mut w = writer.lock().unwrap();
        w.write_all(MAGIC)?;
        w.flush()?;
    }
    let cancelled = Arc::new(AtomicU64::new(0));
    let (tx, rx) = mpsc::channel::<ClientMsg>();
    let flag = cancelled.clone();
    std::thread::spawn(move || {
        let mut reader = reader;
        while let Ok(Some(msg)) = read_frame::<_, ClientMsg>(&mut reader) {
            match msg {
                ClientMsg::Cancel { id } => flag.store(id, Ordering::SeqCst),
                other => {
                    if tx.send(other).is_err() {
                        break;
                    }
                }
            }
        }
    });

    let send = |msg: &WorkerMsg| write_frame(&mut *writer.lock().unwrap(), msg);
    // the Hello comes first
    match rx.recv() {
        Ok(ClientMsg::Hello { .. }) => send(&WorkerMsg::Ready { protocol: PROTOCOL, key: WORKER_KEY.to_string(), device: device_info() })?,
        Ok(_) => return Err(io::Error::new(io::ErrorKind::InvalidData, "the first message is not a Hello")),
        Err(_) => return Ok(()),
    }
    while let Ok(msg) = rx.recv() {
        let ClientMsg::Job { id, job } = msg else {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "unexpected message"));
        };
        let reply = run_job(id, job, &prepare, &cancelled, &writer);
        send(&reply)?;
    }
    Ok(())
}

fn run_job<W: Write + Send + 'static>(
    id: u64,
    job: Job,
    prepare: &impl Fn(Analysis) -> Analysis,
    cancelled: &Arc<AtomicU64>,
    writer: &Arc<Mutex<W>>,
) -> WorkerMsg {
    let cancel = cancelled.clone();
    let out = writer.clone();
    let last = Mutex::new(None::<Instant>);
    let sink = move |p: &bowshock_core::Progress| {
        if cancel.load(Ordering::SeqCst) == id {
            return ControlFlow::Break(());
        }
        let mut last = last.lock().unwrap();
        if last.is_none_or(|t| t.elapsed() >= PROGRESS_INTERVAL) {
            *last = Some(Instant::now());
            if write_frame(&mut *out.lock().unwrap(), &WorkerMsg::Progress(*p)).is_err() {
                return ControlFlow::Break(());
            }
        }
        ControlFlow::Continue(())
    };
    let run = || -> Result<WorkerMsg, Error> {
        let Job::Solve { spec, conditions } = job;
        let analysis = prepare(Analysis::from_spec(&spec)?).on_progress(sink);
        let mut polar = analysis.run(&conditions)?;
        // the final fields go in frames of their own, ahead of the result: a half-domain field of a few million cells is tens of megabytes
        for (point, p) in polar.points.iter_mut().enumerate() {
            if let Some(field) = p.field3.take() {
                match write_frame(&mut *writer.lock().unwrap(), &WorkerMsg::PointField { point, field: Box::new(field) }) {
                    Ok(()) => {}
                    // a field that does not fit a frame is left out; the point stays
                    Err(e) if e.kind() == io::ErrorKind::InvalidData => {}
                    Err(e) => return Err(Error::Internal(format!("sending a field failed: {e}"))),
                }
            }
        }
        Ok(WorkerMsg::Solve(polar))
    };
    match catch_unwind(AssertUnwindSafe(run)) {
        Ok(Ok(msg)) => msg,
        Ok(Err(e)) => WorkerMsg::from_error(&e),
        Err(_) => WorkerMsg::from_error(&Error::Internal("the solver panicked".into())),
    }
}
