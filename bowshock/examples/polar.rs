//! Drag polar of a 0.10 m tangent-ogive body: 0.30 m nose, 0.60 m tube, 0.10 m boattail to 0.08 m.

use bowshock::analysis::{Analysis, Resolution};
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};

fn main() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let polar = Analysis::new(&spec).expect("invalid body").resolution(Resolution::Coarse).run(&[0.6, 0.9, 1.2]).expect("analysis failed");
    println!("mach,cd_forebody,cd_base,cd_total,stop_reason,quality,warnings");
    for p in &polar.points {
        let warnings: Vec<String> = p.warnings.iter().map(|w| format!("{w:?}")).collect();
        println!(
            "{},{:.6},{:.6},{:.6},{},{:?},{}",
            p.mach,
            p.drag.forebody_pressure,
            p.drag.base,
            p.drag.total,
            p.detail.stop_reason,
            p.quality,
            warnings.join("; ")
        );
    }
    println!("backend: {:?}", polar.backend);
    if let Some(note) = &polar.fallback {
        println!("{note}");
    }
}
