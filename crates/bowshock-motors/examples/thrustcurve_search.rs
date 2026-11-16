//! Searches ThrustCurve.org and downloads the first result, printing what came back (a live check of the client).
//!
//! ```text
//! cargo run -p bowshock-motors --features thrustcurve --example thrustcurve_search -- H128
//! ```

use std::time::Instant;

use bowshock_motors::thrustcurve::{self, Client};

fn main() {
    let text = std::env::args().skip(1).collect::<Vec<_>>().join(" ");
    let text = if text.trim().is_empty() { "H128".to_string() } else { text };
    let client = Client::new();
    let start = Instant::now();
    let found = match client.search_text(&text) {
        Ok(found) => found,
        Err(e) => {
            eprintln!("search {text:?}: {e}");
            std::process::exit(1);
        }
    };
    println!("search {text:?}: {} results in {:.2} s ({})", found.len(), start.elapsed().as_secs_f64(), thrustcurve::CREDIT);
    for m in &found {
        println!(
            "  {:<12} {:<10} Ø{:.0} × {:.0} mm  {:>7.1} N·s  {:<10} {} data files  {}",
            m.designation,
            m.manufacturer,
            m.diameter * 1000.0,
            m.length * 1000.0,
            m.total_impulse,
            m.availability,
            m.data_files,
            m.motor_id
        );
    }
    let Some(first) = found.iter().find(|m| m.data_files > 0) else {
        return;
    };
    let start = Instant::now();
    match client.download(std::slice::from_ref(first)) {
        Ok(motors) => {
            for m in motors {
                let info = m.thrustcurve.as_ref().expect("downloaded motors carry their listing");
                println!(
                    "download {} in {:.2} s: id {}, {} points, {:.1} N·s integrated (listed {:.1}), burn {:.2} s, total {:.1} g, prop {:.1} g, delays {:?}, data {} ({}), cert {}",
                    m.designation,
                    start.elapsed().as_secs_f64(),
                    m.id,
                    m.points.len(),
                    m.total_impulse(),
                    info.listed_impulse,
                    m.burn_time(),
                    m.total_mass * 1000.0,
                    m.propellant_mass * 1000.0,
                    m.delays,
                    info.data_source,
                    info.simfile_id,
                    info.cert_org
                );
            }
        }
        Err(e) => {
            eprintln!("download {}: {e}", first.designation);
            std::process::exit(1);
        }
    }
}
