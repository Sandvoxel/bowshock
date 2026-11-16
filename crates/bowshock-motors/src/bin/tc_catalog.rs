//! Builds the bundled motor catalogue from ThrustCurve.org: every listed motor with a data file, its best curve and its
//! metadata, written as a [`MotorFile`] (default `crates/bowshock-motors/data/thrustcurve.bin`).
//!
//! ```text
//! cargo run --release -p bowshock-motors --features thrustcurve --bin tc_catalog -- [OUT]
//! ```
//!
//! Requests are spaced by the client's [`MIN_REQUEST_INTERVAL`](bowshock_motors::thrustcurve::MIN_REQUEST_INTERVAL) and
//! back off on `429`, so a full run takes a few minutes.

use std::collections::BTreeMap;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use bowshock_motors::thrustcurve::{Client, RemoteMotor, SearchQuery, TcError};
use bowshock_motors::{today, MotorFile, THRUSTCURVE_ATTRIBUTION};

/// Impulse classes to list by (the fractional classes are separate criteria).
const CLASSES: &[&str] = &["1/8A", "1/4A", "1/2A", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O"];
/// Motors per download request.
const BATCH: usize = 20;

fn all(criteria: SearchQuery) -> SearchQuery {
    SearchQuery { availability: Some("all".into()), max_results: Some(10_000), ..criteria }
}

/// Runs `f`, retrying a transport failure twice after a pause.
fn retry<T>(what: &str, mut f: impl FnMut() -> Result<T, TcError>) -> Result<T, TcError> {
    let mut tries = 0;
    loop {
        match f() {
            Err(TcError::Unreachable(why)) if tries < 2 => {
                tries += 1;
                eprintln!("  {what}: {why}; retrying");
                std::thread::sleep(Duration::from_secs(5));
            }
            other => return other,
        }
    }
}

fn run(out: &str) -> Result<(), TcError> {
    let start = Instant::now();
    let client = Client::with_timeout(Duration::from_secs(90));
    let makers = retry("metadata", || client.manufacturers().map(<[_]>::to_vec))?;
    eprintln!("{} manufacturers", makers.len());

    let mut listed: BTreeMap<String, RemoteMotor> = BTreeMap::new();
    let mut add = |what: String, found: Vec<RemoteMotor>| {
        let before = listed.len();
        let n = found.len();
        for m in found {
            listed.entry(m.motor_id.clone()).or_insert(m);
        }
        eprintln!("  {what}: {n} motors ({} new)", listed.len() - before);
    };
    for class in CLASSES {
        let q = all(SearchQuery { impulse_class: Some((*class).into()), ..Default::default() });
        match retry(class, || client.search(&q)) {
            Ok(found) => add(format!("class {class}"), found),
            Err(e @ (TcError::Api(_) | TcError::Http(400))) => eprintln!("  class {class}: {e}"),
            Err(e) => return Err(e),
        }
    }
    for (name, abbrev) in &makers {
        let q = all(SearchQuery { manufacturer: Some(abbrev.clone()), ..Default::default() });
        match retry(name, || client.search(&q)) {
            Ok(found) => add(name.clone(), found),
            Err(e @ (TcError::Api(_) | TcError::Http(400))) => eprintln!("  {name}: {e}"),
            Err(e) => return Err(e),
        }
    }
    let (with_data, without): (Vec<RemoteMotor>, Vec<RemoteMotor>) = listed.into_values().partition(|m| m.data_files > 0);
    eprintln!("{} motors listed, {} with data files, {} without", with_data.len() + without.len(), with_data.len(), without.len());

    let mut motors = Vec::with_capacity(with_data.len());
    let batches = with_data.len().div_ceil(BATCH);
    for (i, batch) in with_data.chunks(BATCH).enumerate() {
        let got = retry("download", || client.download(batch))?;
        for meta in batch.iter().filter(|m| !got.iter().any(|g| g.thrustcurve_id() == Some(m.motor_id.as_str()))) {
            eprintln!("  no usable curve: {} {} ({})", meta.manufacturer, meta.designation, meta.motor_id);
        }
        motors.extend(got);
        eprintln!("batch {}/{batches}: {} motors so far ({:.0} s)", i + 1, motors.len(), start.elapsed().as_secs_f64());
    }
    motors.sort_by(|a, b| {
        (a.manufacturer.as_str(), a.total_impulse(), a.designation.as_str())
            .partial_cmp(&(b.manufacturer.as_str(), b.total_impulse(), b.designation.as_str()))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let file = MotorFile { source: "ThrustCurve.org".into(), attribution: THRUSTCURVE_ATTRIBUTION.into(), snapshot: today(), motors };
    let bytes = file.to_bytes();
    std::fs::write(out, &bytes).map_err(|e| TcError::Unreachable(format!("cannot write {out}: {e}")))?;
    println!("wrote {out}: {} motors, {} bytes, snapshot {} ({:.0} s)", file.motors.len(), bytes.len(), file.snapshot, start.elapsed().as_secs_f64());
    Ok(())
}

fn main() -> ExitCode {
    let out = std::env::args().nth(1).unwrap_or_else(|| concat!(env!("CARGO_MANIFEST_DIR"), "/data/thrustcurve.bin").to_string());
    match run(&out) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("tc_catalog: {e}");
            ExitCode::FAILURE
        }
    }
}
