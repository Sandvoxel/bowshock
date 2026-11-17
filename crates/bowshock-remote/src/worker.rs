//! The worker side: the serve loop of `bowshock-worker serve`.

use crate::proto::DeviceInfo;

/// Serves jobs on stdin/stdout until the client hangs up.
pub fn serve_stdio() -> std::io::Result<()> {
    Err(std::io::Error::other("not implemented"))
}

/// `bowshock-worker probe`: the device as one JSON line.
pub fn probe_json() -> String {
    let _: Option<DeviceInfo> = None;
    String::new()
}
