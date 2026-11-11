//! Drag polar of a 0.10 m tangent-ogive body: 0.30 m nose, 0.60 m tube, 0.10 m boattail to 0.08 m. First the inviscid
//! (Euler, pressure drag with a modelled base drag) sweep, then the viscous one (RANS k-omega SST, sea-level standard air).

use bowshock::analysis::{Analysis, Polar, Resolution};
use bowshock::atmosphere::{Atmosphere, FlightConditions};
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};

const MACHS: [f64; 3] = [0.6, 0.9, 1.2];

fn warnings(polar: &Polar, k: usize) -> String {
    polar.points[k].warnings.iter().map(|w| format!("{w:?}")).collect::<Vec<_>>().join("; ")
}

fn backend(polar: &Polar) {
    println!("backend: {:?}", polar.backend);
    if let Some(note) = &polar.fallback {
        println!("{note}");
    }
}

fn main() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let analysis = Analysis::new(&spec).expect("invalid body").resolution(Resolution::Coarse);

    let polar = analysis.clone().run(&MACHS).expect("analysis failed");
    println!("mach,cd_forebody,cd_base,cd_total,stop_reason,quality,warnings");
    for (k, p) in polar.points.iter().enumerate() {
        println!(
            "{},{:.6},{:.6},{:.6},{},{:?},{}",
            p.mach,
            p.drag.forebody_pressure,
            p.drag.base,
            p.drag.total,
            p.detail.stop_reason,
            p.quality,
            warnings(&polar, k)
        );
    }
    backend(&polar);

    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let polar = analysis.viscous(flight).run(&MACHS).expect("viscous analysis failed");
    println!("mach,cd_pressure,cd_friction,cd_base,cd_total,quality,y_plus_max");
    for p in &polar.points {
        println!(
            "{},{:.6},{:.6},{:.6},{:.6},{:?},{:.2}",
            p.mach,
            p.drag.forebody_pressure,
            p.drag.friction.expect("viscous analysis reports friction"),
            p.drag.base,
            p.drag.total,
            p.quality,
            p.detail.y_plus_max
        );
    }
    backend(&polar);
}
