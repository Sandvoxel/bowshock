//! The power-on drag the flight flies: what the solved jet does to the apogee against the build-up's exit-area correction. CPU only (the table's numbers come from a solve; the
//! GPU tests of `perf` print them and call [`print_apogee_effect`]).

use bowshock_model::{CfdGrid, CfdPowerOn, CfdPowerPoint, CfdQuality};
use std::sync::Arc;

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
        println!("  jet effect M {:.2}: friction x{:.3} pressure x{:.3} base x{:.3}", r.mach, r.friction, r.pressure, r.base);
    }
}

fn point(mach: f64, base: f64) -> CfdPowerPoint {
    CfdPowerPoint { mach, friction: 1.0, pressure: 1.0, base, quality: CfdQuality::Converged }
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
    stored.points = [0.1, 0.5, 0.9, 1.3, 2.3, 3.0]
        .iter()
        .map(|&m| CfdPoint { mach: m, cd: 0.6, friction: 0.15, pressure: 0.12, base: 0.2, fins: 0.13, other: 0.0, ..first })
        .collect();
    let mut table = stored;
    let correction = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    let on = |base: f64| CfdPowerOn { jet, solved_at: 0, solve_seconds: 1.0, points: vec![point(0.5, base), point(0.9, base)], notes: Vec::new() };
    table.power_on = Some(on(1.0));
    let all = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    table.power_on = Some(on(0.25));
    let quarter = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&jet)))).0;
    let other = bowshock_model::CfdJet { exit_diameter: jet.exit_diameter * 1.1, ..jet };
    let stale = fly(&rocket, Some(DragTable::from_cfd(&table, Some(&other)))).0;
    assert!(quarter > all + 1.0, "{quarter} m against {all} m");
    assert!((stale - correction).abs() < 1e-9, "a table solved for another jet flies as one without: {stale} against {correction}");
}

/// The power-on part of `plan`'s motor solved as the app does (`solve_power_node`: the body alone with the jet on over the same without it, on the grid of `grid` sized for the
/// highest Mach number of `table`) at the nodes the app takes for `table`, printing the time of each; the points.
pub(super) fn solve_ratios(label: &str, plan: &Plan, table: &CfdTable, grid: CfdGrid) -> Vec<CfdPowerPoint> {
    use std::sync::atomic::AtomicBool;
    let machs = power_on_nodes(&table.points.iter().map(|p| p.mach).collect::<Vec<_>>());
    let job = Job {
        plan: plan.clone(),
        altitude: table.altitude,
        nodes: table.points.iter().map(|p| p.mach).collect(),
        stage: 1,
        grid,
        compute: Default::default(),
        cpu_only: false,
        stability: Vec::new(),
        power_on: machs.clone(),
    };
    let grid_mach = job.nodes.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mut session = Session::Local { backend: solver::Backend::Gpu };
    let cancel = Arc::new(AtomicBool::new(false));
    let t0 = std::time::Instant::now();
    let points: Vec<CfdPowerPoint> = machs
        .iter()
        .filter_map(|&m| {
            let t = std::time::Instant::now();
            let node = solve_power_node(&job, grid_mach, true, m, &mut session, &cancel).unwrap_or_else(|_| panic!("the power-on node at Mach {m}"));
            println!("RATIOS {label} {grid:?} M {m:.2}: {:.1} s, {:?}", t.elapsed().as_secs_f64(), node.point);
            node.point
        })
        .collect();
    println!("RATIOS {label} {grid:?}: {:.1} s for {} nodes", t0.elapsed().as_secs_f64(), points.len());
    points
}

/// The long-burn case: the example rocket with `designation` of `maker` (the same 29 mm case) in its mount. Its power-on part is solved at the table's own nodes and put on a copy
/// of `table`, the power-off table of the example (the geometry is the same: only the motor changes), and flown. The apogee numbers of the long burn, with the example's table.
pub(super) fn long_burn(rocket: &Rocket, table: &CfdTable, maker: &str, designation: &str, grid: CfdGrid) {
    let db = MotorDb::bundled();
    let motor = db
        .motors
        .iter()
        .find(|m| m.manufacturer == maker && m.designation.contains(designation))
        .unwrap_or_else(|| panic!("{maker} {designation} is not bundled"))
        .clone();
    let mut other = rocket.clone();
    let mount = other.motor_mounts().first().cloned().expect("a motor mount");
    other.set_motor(other.active_config_id(), mount.id, Some(from_model::motor_ref(&motor))).expect("the motor fits the mount");
    let vehicle = from_model::vehicle(&other, &db, Phase::Full).expect("the rocket flies");
    let plan = plan(&other, &vehicle, crate::units::UnitSystem::default()).expect("the plan");
    let jet = plan.jet().expect("a jet");
    println!(
        "LONG BURN {} {}: burn {:.2} s, impulse {:.0} N s, average {:.0} N, jet exit {:.1} mm at {:.2} bar",
        motor.manufacturer,
        motor.designation,
        motor.burn_time(),
        motor.total_impulse(),
        motor.average_thrust(),
        1e3 * jet.exit_diameter,
        1e-5 * jet.exit_pressure_pa
    );
    let points = solve_ratios("long burn", &plan, table, grid);
    let table = CfdTable { power_on: Some(CfdPowerOn { jet, solved_at: 0, solve_seconds: 0.0, points, notes: Vec::new() }), ..table.clone() };
    print_apogee_effect(&format!("long burn {} {}", motor.manufacturer, motor.designation), &other, &table, Some(&jet));
}

/// The example's motor and body, and the bundled motors of its diameter with a long burn (candidates for the long-burn case).
#[test]
#[ignore = "prints the example's motor and the long-burn candidates"]
fn print_example_motor_and_long_burn_candidates() {
    let (rocket, vehicle) = example();
    let motor = vehicle.stages[0].motor.as_ref().unwrap();
    let c = &motor.curve;
    println!(
        "EXAMPLE motor d {:.1} mm, length {:.0} mm, burn {:.2} s, impulse {:.0} N s, average {:.0} N, max {:.0} N, mass {:.0} g; body base d {:.1} mm, reference d {:.1} mm, length {:.2} m",
        1e3 * c.diameter,
        1e3 * c.length,
        c.burn_time(),
        c.total_impulse(),
        c.total_impulse() / c.burn_time(),
        c.points.iter().map(|p| p.1).fold(0.0, f64::max),
        1e3 * c.total_mass,
        2e3 * vehicle.body.last().unwrap().aft_radius,
        1e3 * vehicle.reference_diameter,
        vehicle.length()
    );
    println!("PLAN jet {:?}", plan(&rocket, &vehicle, crate::units::UnitSystem::default()).unwrap().motor);
    for m in &MotorDb::bundled().motors {
        let (d, t) = (m.diameter, m.burn_time());
        if (d - c.diameter).abs() < 1e-4 && t >= 3.5 {
            println!(
                "CANDIDATE {} {} d {:.0} mm burn {:.2} s impulse {:.0} N s average {:.0} N delay {:?}",
                m.manufacturer,
                m.designation,
                1e3 * d,
                t,
                m.total_impulse(),
                m.average_thrust(),
                m.delays
            );
        }
    }
}

/// The long-burn motor of the GPU study swaps into the example's mount, flies, and gives its own jet (its table's power-on part is for the example's motor only).
#[test]
fn the_long_burn_motor_swaps_in_flies_and_has_its_own_jet() {
    let (rocket, vehicle) = example();
    let db = MotorDb::bundled();
    let motor = db.motors.iter().find(|m| m.manufacturer == "RATT" && m.designation.contains("I90L")).expect("bundled").clone();
    let mut other = rocket.clone();
    let mount = other.motor_mounts().first().cloned().unwrap();
    other.set_motor(other.active_config_id(), mount.id, Some(from_model::motor_ref(&motor))).unwrap();
    let flown = from_model::vehicle(&other, &db, Phase::Full).unwrap();
    assert!(flown.stages[0].motor.as_ref().unwrap().curve.burn_time() > 6.0);
    let units = crate::units::UnitSystem::default();
    let (a, b) = (plan(&rocket, &vehicle, units).unwrap(), plan(&other, &flown, units).unwrap());
    assert_eq!(a.geometry, b.geometry, "only the motor changes");
    assert!(!a.jet().unwrap().same_as(&b.jet().unwrap()));
    assert!(fly(&other, None).0 > 100.0);
}
