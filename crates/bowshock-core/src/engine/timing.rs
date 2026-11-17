//! Per-point timing of the GPU runners ([`PointTiming`], [`record_timing`], [`take_timings`]), shared by the 2D and 3D runners; printed under `BOWSHOCK_TIMING`.

use std::sync::Mutex;

/// Where the time of one GPU point went (instrumentation only; no numerics). Every [`GpuRunner::run_on`] / `GpuRunner3::run_on` pushes
/// one record to a process-wide log read with [`take_timings`], and prints it as a `TIMING {json}` line on stderr when the environment
/// variable `BOWSHOCK_TIMING` is set.
#[derive(Clone, Debug, Default)]
#[doc(hidden)]
pub struct PointTiming {
    /// 2 or 3 (axisymmetric / 3D runner).
    pub dim: u8,
    pub mach: f64,
    pub alpha_deg: f64,
    /// Warm start on a kept solver (then `build_s`, `ctx_s`, `upload_s` are those of the cold point that created it, not paid here).
    pub warm: bool,
    pub cells: usize,
    /// Host grid + host solver construction (cold points only).
    pub build_s: f64,
    /// `luma::default_context()`.
    pub ctx_s: f64,
    /// `from_host`: device arrays and geometry upload.
    pub upload_s: f64,
    /// `set_config`, `reset` or ramp restart, and the sync.
    pub reset_s: f64,
    /// The first chunk of `SYNC` iterations (includes the trace / kernel compile of a cold solver).
    pub first_chunk_s: f64,
    /// Chunks after the first: count and total seconds (kernels + residual read-back).
    pub chunks_rest: usize,
    pub rest_chunks_s: f64,
    /// State downloads (count, seconds) and the host force / wall / monitor evaluation (seconds) of the monitor samples.
    pub downloads: usize,
    pub download_s: f64,
    pub forces_s: f64,
    /// Final download plus the host post-processing (`finish`, forces).
    pub final_s: f64,
    /// Whole `run_on`.
    pub run_s: f64,
    pub iters_stop: usize,
    pub iters_exec: usize,
    pub stop: String,
}

static TIMINGS: Mutex<Vec<PointTiming>> = Mutex::new(Vec::new());

/// Logs `t` (and prints it under `BOWSHOCK_TIMING`).
#[doc(hidden)]
pub fn record_timing(t: PointTiming) {
    if std::env::var_os("BOWSHOCK_TIMING").is_some() {
        eprintln!(
            "TIMING {{\"dim\":{},\"mach\":{},\"alpha_deg\":{:.1},\"warm\":{},\"cells\":{},\"build_s\":{:.4},\"ctx_s\":{:.4},\"upload_s\":{:.4},\"reset_s\":{:.4},\"first_chunk_s\":{:.4},\"chunks_rest\":{},\"rest_chunks_s\":{:.4},\"downloads\":{},\"download_s\":{:.4},\"forces_s\":{:.4},\"final_s\":{:.4},\"run_s\":{:.4},\"iters_stop\":{},\"iters_exec\":{},\"stop\":\"{}\"}}",
            t.dim, t.mach, t.alpha_deg, t.warm, t.cells, t.build_s, t.ctx_s, t.upload_s, t.reset_s, t.first_chunk_s, t.chunks_rest, t.rest_chunks_s, t.downloads, t.download_s, t.forces_s, t.final_s, t.run_s, t.iters_stop, t.iters_exec, t.stop
        );
    }
    TIMINGS.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push(t);
}

/// Drains the timing log.
#[doc(hidden)]
pub fn take_timings() -> Vec<PointTiming> {
    std::mem::take(&mut *TIMINGS.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}
