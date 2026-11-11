//! Drag polar of a fineness-3 tangent ogive with a 6 d cylinder.

use bowshock::analysis::{Analysis, Resolution};
use bowshock::body_grid::{BaseBody, Body, Nose};

fn main() {
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    let polar = Analysis::from_internal(body).resolution(Resolution::Coarse).run(&[0.6, 0.9, 1.2]).expect("analysis failed");
    println!("mach,cd_forebody,cd_base,cd_total,stop_reason");
    for p in &polar.points {
        println!("{},{:.6},{:.6},{:.6},{}", p.mach, p.drag.forebody_pressure, p.drag.base, p.drag.total, p.detail.stop_reason);
    }
    println!("backend: {:?}", polar.backend);
    if let Some(note) = &polar.fallback {
        println!("{note}");
    }
}
