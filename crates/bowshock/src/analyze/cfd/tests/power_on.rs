//! The power-on drag the flight flies: what the solved jet does to the apogee against the build-up's exit-area correction. CPU only (the table's numbers come from a solve; the
//! GPU tests of `perf` print them and call [`print_apogee_effect`]).

use bowshock_model::{CfdPowerOn, CfdPowerPoint, CfdQuality};
use bowshock_sim::{simulate, DragTable, Launch};

use super::*;

/// Apogee (m) and Mach number at burnout of the rocket of `rocket` flown on `table` (`None`: the Extended build-up), zero wind, a 1.5 m rail.
pub(super) fn fly(rocket: &Rocket, table: Option<DragTable>) -> (f64, f64) {
    let launch = Launch { rail_length: 1.5, ..Launch::default() };
    let mut input = from_model::sim_input(rocket, &MotorDb::bundled(), Phase::Full, launch).expect("the rocket flies");
    if let Some(t) = table {
        input.drag_tables = vec![Some(t)];
    }
    let r = simulate(&input);
    let burnout = input.vehicle.stages[0].motor.as_ref().map_or(0.0, |m| m.curve.burn_time());
    let mach_at_burnout = r.trajectory.samples.iter().find(|s| s.t >= burnout).map_or(f64::NAN, |s| s.mach);
    (r.summary.apogee, mach_at_burnout)
}

/// Prints the apogee of `rocket` on the Extended build-up, on `table` with the build-up's exit-area correction for the motor on (`from_cfd` without the jet) and on `table` with
/// its solved jet (`jet` is the motor's now). The CPU half of the power-on measurements.
pub(super) fn print_apogee_effect(label: &str, rocket: &Rocket, table: &CfdTable, jet: Option<&bowshock_model::CfdJet>) {
    let (extended, m_burnout) = fly(rocket, None);
    let (exit_area, _) = fly(rocket, Some(DragTable::from_cfd(table, None)));
    let (solved, _) = fly(rocket, Some(DragTable::from_cfd(table, jet)));
    let on = DragTable::from_cfd(table, jet);
    println!(
        "APOGEE {label}: Extended {extended:.1} m, table with the exit-area correction {exit_area:.1} m, table with the solved jet {solved:.1} m ({:+.2} m, {:+.2} % of the correction's apogee); Mach {m_burnout:.2} at burnout; {} power-on nodes",
        solved - exit_area,
        100.0 * (solved / exit_area - 1.0),
        on.power_on.len()
    );
    for r in &on.power_on {
        println!("  jet effect M {:.2}: friction {:.3} pressure {:.3} base {:.3} fins {:.3}", r.mach, r.friction, r.pressure, r.base, r.fins);
    }
}

fn point(mach: f64, base: f64) -> CfdPowerPoint {
    CfdPowerPoint { mach, friction: 0.12, pressure: 0.12, base, fins: 0.12, quality: CfdQuality::Converged }
}

/// The jet's effect decides the apogee (a jet that keeps a quarter of the base drag flies higher than one that keeps all of it), and the jet's identity decides whether the
/// table's power-on part is used: a table solved for another motor flies on the exit-area correction, the same apogee as a table without a power-on part.
#[test]
fn the_example_flies_higher_on_a_jet_that_lowers_the_base_drag() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, crate::units::UnitSystem::default()).unwrap();
    let jet = plan.jet().expect("the example has a motor");
    let mut stored = table();
    let first = stored.points[0];
    stored.altitude = 0.0;
    stored.points = [0.1, 0.5, 0.9, 1.3, 2.3, 3.0].iter().map(|&m| CfdPoint { mach: m, cd: 0.6, friction: 0.15, pressure: 0.12, base: 0.2, fins: 0.13, other: 0.0, ..first }).collect();
    let mut table = stored;
    let correction = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    let on = |base: f64| CfdPowerOn { jet, solved_at: 0, solve_seconds: 1.0, points: vec![point(0.5, base), point(0.9, base)], notes: Vec::new() };
    table.power_on = Some(on(0.2));
    let all = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    table.power_on = Some(on(0.05));
    let quarter = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    let other = bowshock_model::CfdJet { exit_diameter: jet.exit_diameter * 1.1, ..jet };
    let stale = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&other)))).0;
    assert!(quarter > all + 1.0, "{quarter} m against {all} m");
    assert!((stale - correction).abs() < 1e-9, "a table solved for another jet flies as one without: {stale} against {correction}");
}
