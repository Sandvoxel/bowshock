//! Runs bowshock solves on another machine over `ssh`: the app (or any program) calls [`Remote::connect`], which installs the
//! worker on the remote machine when it is missing (building it from the embedded source set), and then runs solve jobs (the drag sweep and the 3D conditions at an angle of attack of `Analysis::run`)
//! on that machine's GPU, with progress and cancellation, as [`bowshock_core::Analysis`] does locally.
//!
//! * [`proto`]: the wire protocol (CBOR frames over the ssh stdin/stdout).
//! * [`client`]: [`Remote`], [`RemoteHost`], [`RemoteError`], [`test_connection`].
//! * [`provision`]: the worker's version key, and installing it on the remote.
//! * [`worker`]: the worker's serve loop (the `bowshock-worker` binary runs it).

#![warn(missing_docs)]

pub mod client;
pub mod proto;
pub mod provision;
pub mod worker;

#[doc(hidden)]
pub mod sourceset;

pub use client::{test_connection, Remote, RemoteError, RemoteHost};
pub use proto::{DeviceInfo, Job};
pub use provision::SetupStatus;

/// Source key of the worker this crate was built together with (hex SHA-256 over its source set; `unavailable` when built outside
/// the workspace).
pub const WORKER_KEY: &str = include_str!(concat!(env!("OUT_DIR"), "/worker-key.txt"));
/// The source set of the worker as a gzip tar archive (empty when the key came from `BOWSHOCK_WORKER_KEY` or the workspace was
/// not there).
pub const WORKER_SOURCE: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/worker-src.tar.gz"));
