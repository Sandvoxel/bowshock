//! `bowshock-worker serve` (jobs over stdin/stdout, started by `bowshock-remote` through ssh) and `bowshock-worker probe` (one JSON
//! line about this machine's device).

use std::process::ExitCode;

fn main() -> ExitCode {
    match std::env::args().nth(1).as_deref() {
        Some("serve") => match bowshock_remote::worker::serve_stdio() {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock-worker: {e}");
                ExitCode::FAILURE
            }
        },
        Some("probe") => {
            println!("{}", bowshock_remote::worker::probe_json());
            ExitCode::SUCCESS
        }
        _ => {
            eprintln!("usage: bowshock-worker serve | probe");
            ExitCode::from(2)
        }
    }
}
