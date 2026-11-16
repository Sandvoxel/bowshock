use bowshock_core as solver;
use bowshock_model::{CfdGrid, CfdLiftTable, CfdPoint, CfdQuality, CfdTable, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Phase};
use bowshock_sim::Vehicle;

use super::*;

mod interp;
mod openrocket;
mod plan;
mod run;

const EXAMPLE: &[u8] = include_bytes!("../../../../assets/examples/hpr_l1_cert.bowshock");

pub(super) fn example() -> (Rocket, Vehicle) {
    let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
    let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
    (rocket, vehicle)
}
pub(super) fn lift_table(key: u64) -> CfdLiftTable {
    // CN(2°) = 0.1 and 0.2 at M 0.8 and 2.0; CN(6°) = 0.4 and 0.8; CP 1.0 / 1.4 m at 2°, 1.1 / 1.5 m at 6°
    let p = |mach: f64, alpha_deg: f64, cn: f64, x_cp: f64| bowshock_model::CfdLiftPoint { mach, alpha_deg, cn, x_cp, quality: CfdQuality::Converged };
    CfdLiftTable {
        key,
        solved_at: 1,
        solve_seconds: 60.0,
        backend: "GPU (Luma)".into(),
        points: vec![p(0.8, 2.0, 0.1, 1.0), p(0.8, 6.0, 0.4, 1.1), p(2.0, 2.0, 0.2, 1.4), p(2.0, 6.0, 0.8, 1.5)],
        notes: Vec::new(),
    }
}
pub(super) fn no_ext(_: f64) -> Option<ExtStab> {
    None
}
pub(super) fn raw(mach: f64, alpha_deg: f64, q: solver::Quality) -> solver::LiftPoint {
    solver::LiftPoint { mach, alpha: alpha_deg.to_radians(), cn: 0.3, ca_pressure: 0.1, x_cp: 1.0, quality: q, iterations: 10 }
}
pub(super) fn table() -> CfdTable {
    let p = |mach: f64, cd: f64| CfdPoint {
        mach,
        cd,
        friction: cd * 0.2,
        pressure: cd * 0.2,
        base: cd * 0.2,
        fins: cd * 0.3,
        other: cd * 0.1,
        quality: CfdQuality::Converged,
    };
    CfdTable {
        key: 1,
        geometry: 1,
        stage: 1,
        solved_at: now(),
        solve_seconds: 1.0,
        backend: "GPU (Luma)".into(),
        fallback: None,
        altitude: 120.0,
        notes: Vec::new(),
        points: vec![p(0.5, 0.4), p(1.0, 0.8), p(2.0, 0.6)],
        grid: CfdGrid::Coarse,
        lift: None,
    }
}
