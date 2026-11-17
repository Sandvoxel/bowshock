//! Installing the worker on the remote machine.

use crate::proto::DeviceInfo;

/// Where [`Remote::connect`](crate::Remote::connect) is while it sets the remote up.
#[derive(Clone, Debug, PartialEq)]
pub enum SetupStatus {
    /// Opening the ssh connection.
    Connecting,
    /// Looking for the worker on the remote.
    Checking,
    /// Sending the worker (binary or source) to the remote.
    Uploading {
        /// Bytes sent.
        bytes: u64,
    },
    /// Building the worker on the remote: the latest line of cargo's output.
    Building {
        /// The line.
        line: String,
    },
    /// Asking the worker about its device.
    Probing,
    /// Connected.
    Ready(DeviceInfo),
}
