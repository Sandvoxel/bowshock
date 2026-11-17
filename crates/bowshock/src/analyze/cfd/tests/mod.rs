use bowshock_core as solver;
use bowshock_model::{CfdGrid, CfdLiftTable, CfdPoint, CfdQuality, CfdTable, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Phase};
use bowshock_sim::Vehicle;

use super::*;

mod interp;
mod openrocket;
mod perf;
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
    let p = |mach: f64, alpha_deg: f64, cn: f64, x_cp: f64| bowshock_model::CfdLiftPoint {
        mach,
        alpha_deg,
        cn,
        x_cp,
        cd: None,
        cm: None,
        quality: CfdQuality::Converged,
    };
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
/// A point of the 3D solver at `alpha_deg`: CN 0.3, CP 1 m.
pub(super) fn raw(mach: f64, alpha_deg: f64, q: solver::Quality) -> solver::PolarPoint {
    solver::PolarPoint {
        mach,
        alpha_deg,
        drag: solver::Drag { forebody_pressure: 0.1, friction: None, base: f64::NAN, base_modelled: false, fins: None, total: f64::NAN },
        forces: Some(solver::Forces { cn: 0.3, cm: -0.3, x_cp: 1.0, ca: 0.1 }),
        detail: solver::PointDetail {
            iterations: 10,
            residual_drop_orders: 3.0,
            stop_reason: solver::StopReason::ResidualDrop,
            cd_nose: f64::NAN,
            cd_aft: Vec::new(),
            cd_friction: 0.0,
            cd_base_solver: f64::NAN,
            cp_base: f64::NAN,
            wall: Vec::new(),
            y_plus_max: f64::NAN,
            farfield_dp: 0.0,
        },
        quality: q,
        warnings: Vec::new(),
        cd_uncertainty: None,
        field: None,
        field3: None,
    }
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
        interference: 0.0,
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
