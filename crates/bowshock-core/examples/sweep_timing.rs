//! Wall time of a realistic sweep: the `polar.rs` body with four rounded fins, viscous (RANS, k-omega SST) at sea level,
//! `Resolution::Medium`, `Backend::Auto`, Mach 0.3 to 2.5 in steps of 0.1, each Mach number run on its own.
//!
//! `cargo run --release -p bowshock-core --example sweep_timing` (GPU when present); `-- --cpu` runs Mach 0.5, 1.0, 2.0 on the CPU.

use bowshock_core::{AftSpec, Analysis, Atmosphere, Backend, BodySpec, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec, Resolution};
use std::time::Instant;

fn main() {
    let cpu = std::env::args().any(|a| a == "--cpu");
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let fins = FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, profile: FinProfile::Rounded };
    let (backend, machs): (Backend, Vec<f64>) =
        if cpu { (Backend::Cpu, vec![0.5, 1.0, 2.0]) } else { (Backend::Auto, (3..=25).map(|k| f64::from(k) / 10.0).collect()) };
    let analysis = Analysis::new(&spec)
        .expect("valid body")
        .fins(fins)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
        .resolution(Resolution::Medium)
        .backend(backend);

    println!("mach,seconds,quality,cd_total");
    let t_all = Instant::now();
    let mut used = None;
    for &m in &machs {
        let t0 = Instant::now();
        let polar = analysis.run(&[m]).expect("analysis failed");
        let p = &polar.points[0];
        println!("{m:.1},{:.2},{:?},{:.5}", t0.elapsed().as_secs_f64(), p.quality, p.drag.total);
        if used.is_none() {
            println!("# backend: {:?}{}", polar.backend, polar.fallback.as_deref().map(|f| format!(" ({f})")).unwrap_or_default());
        }
        used = Some(polar.backend);
    }
    println!("total: {:.1} s for {} points, backend {:?}", t_all.elapsed().as_secs_f64(), machs.len(), used.expect("at least one point"));
}
