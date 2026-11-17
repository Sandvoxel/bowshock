use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use bowshock_model::CfdGrid;
use bowshock_sim::{vs_mach, Atmosphere};

use super::*;
use crate::analyze::solve::{Iteration, Outcome, PointDone, Tracker};
use crate::units::UnitSystem;

#[test]
fn failed_3d_points_are_dropped_and_fewer_than_two_nodes_is_an_error() {
    use solver::Quality::{Converged, Failed, Stalled};
    let mut notes = Vec::new();
    let ok = [raw(0.5, 2.0, Converged), raw(0.5, 6.0, Stalled), raw(0.8, 2.0, Converged), raw(0.8, 6.0, Failed)];
    let pts = reduce_lift(&ok, 2, &mut notes).unwrap();
    assert_eq!(pts.len(), 3);
    assert_eq!(notes.len(), 1);
    assert!((pts[1].alpha_deg - 6.0).abs() < 1e-9 && pts[1].quality == CfdQuality::Stalled);
    // the stored point has the normal force, the centre of pressure and the moment; the Euler solve gives no drag
    assert_eq!((pts[0].cn, pts[0].x_cp, pts[0].cm, pts[0].cd), (0.3, 1.0, Some(-0.3), None));
    // a point without forces (a drag point) or with a non-finite centre of pressure is not a stability point
    let mut no_forces = raw(0.5, 2.0, Converged);
    no_forces.forces = None;
    assert!(reduce_lift(&[no_forces], 1, &mut Vec::new()).is_err());
    let mut vanishing = raw(0.5, 2.0, Converged);
    vanishing.forces.as_mut().unwrap().x_cp = f64::NAN;
    assert!(reduce_lift(&[vanishing], 1, &mut Vec::new()).is_err());

    let one = [raw(0.5, 2.0, Converged), raw(0.5, 6.0, Converged), raw(0.8, 2.0, Failed), raw(0.8, 6.0, Failed)];
    assert!(reduce_lift(&one, 2, &mut Vec::new()).is_err());
    // A single requested node may stand alone; nothing usable is still an error.
    assert_eq!(reduce_lift(&one[..2], 1, &mut Vec::new()).unwrap().len(), 2);
    assert!(reduce_lift(&[raw(0.5, 2.0, Failed)], 1, &mut Vec::new()).is_err());
}

/// One real solve on a tiny grid (the example rocket, two Mach numbers, coarse grid). Needs a GPU for speed; ignored in the
/// default run (about 1 to 2 minutes on an Apple M1 Max, see the task report): `cargo test -p bowshock -- --ignored tiny`.
#[test]
#[ignore = "solves on the GPU: about a minute"]
fn a_tiny_grid_solves_and_is_sane() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let job = Job { plan, altitude: 120.0, nodes: vec![0.6, 2.0], stage: 1, grid: CfdGrid::Coarse, compute: Default::default(), cpu_only: false };
    let points = Arc::new(Mutex::new(0usize));
    let counter = points.clone();
    let cancel = Arc::new(AtomicBool::new(false));
    let t = solve(&job, &cancel, move |e| {
        if matches!(e, Event::Point(_)) {
            *counter.lock().unwrap() += 1;
        }
    })
    .expect("the example solves");
    assert_eq!((*points.lock().unwrap(), t.points.len()), (2, 2));
    for p in &t.points {
        assert!(p.cd > 0.2 && p.cd < 1.0, "a slender rocket's drag coefficient: {p:?}");
        assert!((p.friction + p.pressure + p.base + p.fins + p.other - p.cd).abs() < 1e-9);
    }
    assert_eq!(t.key, table_key(job.plan.geometry, 120.0, &job.nodes));
    println!("solved {} points on {} in {:.1} s: {:?}", t.points.len(), t.backend, t.solve_seconds, t.points);
}

/// Solves the example's whole default table (10 Mach numbers, minutes on an M1 Max), prints it against the Extended method
/// and checks the two agree to within a factor of two with the drag rise in the transonic range. With
/// `BOWSHOCK_CFD_FIXTURE=<path>` it also writes the example document with the table stored (the file a drive script opens
/// with `BOWSHOCK_OPEN`): `BOWSHOCK_CFD_FIXTURE=/tmp/example-cfd.bowshock cargo test -p bowshock -- --ignored the_example_table --nocapture`.
#[test]
#[ignore = "solves ten Mach numbers on the GPU: minutes"]
fn the_example_table_solves_and_agrees_with_extended() {
    let (mut rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let nodes = nodes((0.1, 3.0)).unwrap();
    let job = Job { plan, altitude: 120.0, nodes, stage: 1, grid: CfdGrid::Coarse, compute: Default::default(), cpu_only: false };
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), move |e| {
        if let Event::Point(p) = e {
            println!("{:.0} s point {p:?}", t0.elapsed().as_secs_f64());
        }
    })
    .expect("the example solves");
    let atm = Atmosphere::standard();
    let cg = crate::analyze::compute::jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap().0[0].cg_liftoff;
    println!("Mach | margin Extended | margin CFD | Cd Extended | Cd CFD | quality");
    for p in &table.points {
        let e = &vs_mach(&vehicle, cg, (p.mach, p.mach), 1, 0.0, &atm, 120.0)[0];
        println!("{:.2} | {:.3} | {:.3} | {:.4} | {:.4} | {:?}", p.mach, e.margin_cal, e.margin_cal, e.drag.total, p.cd, p.quality);
        assert!(p.cd > 0.5 * e.drag.total && p.cd < 2.0 * e.drag.total, "Mach {}: CFD {} vs Extended {}", p.mach, p.cd, e.drag.total);
    }
    let peak = table.points.iter().max_by(|a, b| a.cd.total_cmp(&b.cd)).unwrap();
    assert!(peak.mach > 0.8 && peak.mach < 2.0, "the drag rises through the transonic range: peak at Mach {}", peak.mach);
    println!("{} points on {} in {:.0} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
    if let Some(path) = std::env::var_os("BOWSHOCK_CFD_FIXTURE") {
        assert!(rocket.store_cfd_table(table));
        std::fs::write(path, rocket.to_bytes()).unwrap();
    }
}

#[test]
fn a_cancel_stops_before_the_next_point() {
    let (rocket, vehicle) = example();
    let job = Job {
        plan: plan(&rocket, &vehicle, UnitSystem::default()).unwrap(),
        altitude: 120.0,
        nodes: vec![0.6, 2.0],
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
    };
    let cancel = Arc::new(AtomicBool::new(true));
    assert_eq!(solve(&job, &cancel, |_| ()).unwrap_err(), CANCELLED);
}

/// A real solve ended by a cancel from inside the run: the solver sees the flag at its next report (50 iterations), so the
/// point in flight ends within moments, not minutes, and no point is sent. Needs the solver to start (the GPU kernels, or the CPU
/// grid): a few seconds to half a minute.
#[test]
#[ignore = "starts the solver: seconds"]
fn a_cancel_ends_the_point_in_flight_within_a_report() {
    let (rocket, vehicle) = example();
    let job = Job {
        plan: plan(&rocket, &vehicle, UnitSystem::default()).unwrap(),
        altitude: 120.0,
        nodes: vec![1.0, 2.0],
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
    };
    let cancel = Arc::new(AtomicBool::new(false));
    let seen = Arc::new(Mutex::new(Vec::<String>::new()));
    let (flag, log) = (cancel.clone(), seen.clone());
    let t0 = Instant::now();
    let result = solve(&job, &cancel, move |e| {
        let mut log = log.lock().unwrap();
        match e {
            Event::Machine { state: crate::analyze::solve::MachineState::Ready(name), .. } => log.push(format!("device {name}")),
            Event::Iteration(i) => {
                log.push(format!("iteration {} of Mach {} at {:.1} s", i.iteration, i.mach, i.at));
                // the first report cancels
                flag.store(true, Ordering::Relaxed);
            }
            Event::Point(p) => log.push(format!("point {}", p.mach)),
            _ => {}
        }
    });
    println!("{:?} after {:.1} s: {:?}", result.as_ref().err(), t0.elapsed().as_secs_f64(), seen.lock().unwrap());
    assert_eq!(result.unwrap_err(), CANCELLED);
    let seen = seen.lock().unwrap();
    assert!(seen[0].starts_with("device"));
    assert_eq!(seen.iter().filter(|l| l.starts_with("iteration")).count(), 1, "one report, then the run ended");
    assert!(!seen.iter().any(|l| l.starts_with("point")), "a cancelled point is not sent");
}

/// The real event stream of a one-point solve (Mach 2): the device first, iteration reports that go up in 50s at about 10 Hz or
/// slower, then the point, whose iteration count the last report did not pass. Prints what the screen would see.
#[test]
#[ignore = "solves one point: about 30 s"]
fn a_real_point_streams_progress() {
    let (rocket, vehicle) = example();
    let job = Job {
        plan: plan(&rocket, &vehicle, UnitSystem::default()).unwrap(),
        altitude: 120.0,
        nodes: vec![2.0],
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
    };
    let events = Arc::new(Mutex::new(Vec::<Event>::new()));
    let log = events.clone();
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), move |e| log.lock().unwrap().push(e));
    let secs = t0.elapsed().as_secs_f64();
    let events = events.lock().unwrap();
    let reports: Vec<&Iteration> = events.iter().filter_map(|e| if let Event::Iteration(i) = e { Some(i) } else { None }).collect();
    let point = events.iter().find_map(|e| if let Event::Point(p) = e { Some(p) } else { None }).expect("a point was solved");
    println!("{} reports in {secs:.1} s; device {:?}; point {point:?}", reports.len(), events.first());
    for r in reports.iter().step_by((reports.len() / 12).max(1)) {
        println!("  {:.1} s it {} orders {:.1} Cd {:.4} spread {:?} tol {:?}", r.at, r.iteration, r.residual_orders, r.monitor, r.spread, r.tolerance);
    }
    assert!(matches!(events.first(), Some(Event::Machines(_))));
    assert!(reports.windows(2).all(|w| w[1].iteration > w[0].iteration && w[1].at - w[0].at >= REPORT_INTERVAL - 1e-9));
    assert!(reports.iter().all(|r| r.iteration % 50 == 0 && r.mach == 2.0));
    assert!(reports.last().is_some_and(|r| r.iteration <= point.iterations));
    assert!(reports.last().is_some_and(|r| r.monitor.is_finite() && r.monitor > 0.1 && r.monitor < 2.0), "the monitor is a drag coefficient");
    assert_eq!(table.expect("the point solves").points.len(), 1);
}

#[test]
fn reports_go_on_at_about_ten_hertz_and_the_first_of_a_run_always_goes() {
    // the first report of a solve (nothing sent yet)
    assert!(report_due(f64::NEG_INFINITY, 0, 3.2, 50));
    // 40 ms after the last one: held back
    assert!(!report_due(3.2, 50, 3.24, 100));
    // 100 ms after: goes
    assert!(report_due(3.2, 50, 3.3, 100));
    // a new run (the iteration count went back) goes at once, however soon
    assert!(report_due(3.2, 4000, 3.21, 50));
}

#[test]
fn a_table_that_stopped_early_is_a_partial_of_the_range_and_nothing_else_is() {
    let (rocket, vehicle) = example();
    let mapped = plan(&rocket, &vehicle, UnitSystem::default());
    let job = Job {
        plan: mapped.clone().unwrap(),
        altitude: 120.0,
        nodes: nodes((0.1, 3.0)).unwrap(),
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
    };
    let mut tracker = Tracker::new(crate::analyze::solve::Kind::Drag, Tracker::drag_units(&job.nodes));
    let done = |mach: f64, quality: Option<CfdQuality>| PointDone {
        machine: 0,
        at: 10.0,
        mach,
        alpha_deg: None,
        outcome: Outcome::from_quality(quality),
        point: quality.map(|quality| CfdPoint { mach, cd: 0.5, friction: 0.1, pressure: 0.1, base: 0.2, fins: 0.1, other: 0.0, interference: 0.0, quality }),
        lift: None,
        iterations: 2000,
        cpu: false,
        fallback: None,
    };
    tracker.on_done(&done(job.nodes[0], Some(CfdQuality::Converged)));
    assert!(partial_table(&job, &tracker, 30.0).is_none(), "one point is not a curve");
    tracker.on_done(&done(job.nodes[1], None));
    assert!(partial_table(&job, &tracker, 30.0).is_none(), "a fallback is not a point");
    tracker.on_done(&done(job.nodes[2], Some(CfdQuality::Stalled)));
    let table = partial_table(&job, &tracker, 30.0).expect("two solved points make a table");
    assert_eq!(table.points.iter().map(|p| p.mach).collect::<Vec<_>>(), vec![job.nodes[0], job.nodes[2]]);
    assert_eq!(table.key, table_key(job.plan.geometry, 120.0, &job.nodes[..3]));
    assert_eq!(table.solve_seconds, 30.0);
    assert_eq!(table.backend, "GPU (Luma)");
    assert!(table.notes.iter().any(|n| n.contains("did not converge and is interpolated")), "{:?}", table.notes);
    assert!(table.notes.iter().any(|n| n == "Stopped early: 3 of 10 Mach points solved."), "{:?}", table.notes);
    assert!(table.notes.iter().any(|n| n.contains("1 point stopped at the iteration cap")), "{:?}", table.notes);
    // it is a table for the first 3 of the 10 nodes, and stale as the whole table
    assert_eq!(partial_of(&mapped, &table, 120.0, (0.1, 3.0)), Some((3, 10)));
    assert_eq!(status(&mapped, Some(&table), 120.0, (0.1, 3.0)), Status::Stale);
    // a whole table, or one for other conditions, is not a partial one
    let mut whole = table.clone();
    whole.key = table_key(job.plan.geometry, 120.0, &job.nodes);
    assert_eq!(partial_of(&mapped, &whole, 120.0, (0.1, 3.0)), None);
    assert_eq!(partial_of(&mapped, &table, 500.0, (0.1, 3.0)), None, "another altitude");
    // the document keeps it, and the plots can draw it over the Mach numbers it has
    let mut kept = Rocket::default();
    assert!(kept.store_cfd_table(table.clone()));
    assert!(covers(&table, (0.1, 3.0)));
}

#[test]
fn the_backend_text_names_the_machines_that_solved_it() {
    let local = |units, cpu_units| Part { device: "NVIDIA GeForce RTX 3090".into(), remote: false, units, cpu_units };
    let remote = |units| Part { device: "NVIDIA GeForce RTX 3080 Laptop GPU on omarchy".into(), remote: true, units, cpu_units: 0 };
    assert_eq!(backend_text(&[local(10, 0)]), "GPU (Luma)");
    assert_eq!(backend_text(&[local(10, 10)]), "CPU");
    assert_eq!(backend_text(&[local(10, 4)]), "GPU (Luma) and CPU");
    assert_eq!(backend_text(&[remote(10)]), "GPU (Luma) · NVIDIA GeForce RTX 3080 Laptop GPU on omarchy");
    assert_eq!(backend_text(&[local(6, 0), remote(4)]), "GPU (Luma) · NVIDIA GeForce RTX 3090 on this computer, NVIDIA GeForce RTX 3080 Laptop GPU on omarchy");
}

#[test]
fn a_stopped_share_keeps_the_machines_that_solved_it_and_the_part_without_a_gap() {
    use crate::analyze::solve::{MachineInfo, MachineState};
    let job = Job {
        plan: test_plan(),
        altitude: 120.0,
        nodes: vec![0.5, 0.9, 1.1, 1.5],
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
    };
    let mut t = Tracker::new(crate::analyze::solve::Kind::Drag, Tracker::drag_units(&job.nodes));
    t.set_machines(&[MachineInfo { name: "This computer".into(), remote: false }, MachineInfo { name: "omarchy".into(), remote: true }]);
    t.on_machine(0, 0.0, MachineState::Ready("RTX 3090".into()));
    t.on_machine(1, 0.0, MachineState::Ready("RTX 3080 on omarchy".into()));
    let done = |machine: usize, mach: f64| PointDone {
        machine,
        at: 10.0,
        mach,
        alpha_deg: None,
        outcome: Outcome::Converged,
        point: Some(CfdPoint {
            mach,
            cd: 0.5,
            friction: 0.1,
            pressure: 0.1,
            base: 0.2,
            fins: 0.1,
            other: 0.0,
            interference: 0.0,
            quality: CfdQuality::Converged,
        }),
        lift: None,
        iterations: 2000,
        cpu: false,
        fallback: None,
    };
    // Mach 0.5 and 0.9 on this computer, 1.5 on the remote; 1.1 is still running: the table is the first two
    t.on_done(&done(0, 0.5));
    t.on_done(&done(0, 0.9));
    t.on_done(&done(1, 1.5));
    let table = partial_table(&job, &t, 30.0).expect("two points in a row make a table");
    assert_eq!(table.points.iter().map(|p| p.mach).collect::<Vec<_>>(), vec![0.5, 0.9]);
    assert_eq!(table.key, table_key(job.plan.geometry, 120.0, &job.nodes[..2]));
    assert_eq!(table.backend, "GPU (Luma)", "only this computer solved the points kept");
    assert!(table.notes.iter().any(|n| n == "Stopped early: 2 of 4 Mach points solved."), "{:?}", table.notes);
    // with the gap filled by the remote, both machines are named
    t.on_done(&done(1, 1.1));
    let table = partial_table(&job, &t, 30.0).unwrap();
    assert_eq!(table.points.len(), 4);
    assert_eq!(table.backend, "GPU (Luma) · RTX 3090 on this computer, RTX 3080 on omarchy");
}

/// Solves that cannot reach a machine fail with its words and run nothing, on any device.
#[test]
fn solves_that_cannot_reach_a_machine_fail_with_its_error_or_with_every_machines() {
    use crate::app::settings::{Compute, Machine, MachineRef, Mode as ComputeMode};
    let plan = test_plan();
    let machines = vec![Machine::parse("nobody@invalid.invalid"), Machine::parse("nobody@invalid.example:2222")];
    let events = Arc::new(Mutex::new(Vec::<Event>::new()));
    let run = |compute: Compute| {
        events.lock().unwrap().clear();
        let log = events.clone();
        let job = Job { plan: plan.clone(), altitude: 0.0, nodes: vec![0.6, 2.0], stage: 1, grid: CfdGrid::Coarse, compute, cpu_only: false };
        solve(&job, &Arc::new(AtomicBool::new(false)), move |e| log.lock().unwrap().push(e))
    };
    // one machine, picked: its error as it is
    let one = Compute { machines: machines.clone(), mode: ComputeMode::Pick(MachineRef::Machine(machines[0].key())), use_this_computer: false };
    let err = run(one).expect_err("nothing answers there");
    assert!(err.starts_with("Remote GPU nobody@invalid.invalid: "), "{err}");
    assert_ne!(err, CANCELLED);
    assert!(!events.lock().unwrap().iter().any(|e| matches!(e, Event::Point(_) | Event::Iteration(_))), "no solve ran, on any device");
    // shared by two that cannot be reached: the error names both
    let both = Compute { machines: machines.clone(), mode: ComputeMode::Share, use_this_computer: false };
    let err = run(both).expect_err("nothing answers on either");
    assert!(err.starts_with("No machine could finish the solve."), "{err}");
    assert!(err.contains("nobody@invalid.invalid:") && err.contains("nobody@invalid.example:2222:"), "{err}");
    let events = events.lock().unwrap();
    assert!(matches!(events.first(), Some(Event::Machines(m)) if m.len() == 2));
    assert_eq!(events.iter().filter(|e| matches!(e, Event::Machine { state: crate::analyze::solve::MachineState::Skipped(_), .. })).count(), 2);
    // nothing on at all is an error too, and nothing is contacted
    let none = Compute { machines, mode: ComputeMode::Share, use_this_computer: false };
    let mut none = none;
    none.machines.iter_mut().for_each(|m| m.enabled = false);
    drop(events);
    assert!(run(none).unwrap_err().contains("No machine is switched on"));
}

/// The same solves on this computer, on a remote GPU and shared by both agree: the 2D drag table and the 3D stability table of
/// the example (six Mach numbers each) through `solve` and `solve_lift` with this computer alone, `$BOWSHOCK_REMOTE_TEST_HOST`
/// alone and both sharing the work. Cd and CN to 0.5 %, CP to 0.01 cal (the example's reference diameter, 76.2 mm). Prints the wall
/// times, the machines' states and how many points each solved:
/// `BOWSHOCK_REMOTE_TEST_HOST=sandvoxel@192.168.1.88 cargo test --release -p bowshock -- --ignored a_remote_solve --nocapture`.
#[test]
#[ignore = "solves on a local and a remote GPU: minutes; needs BOWSHOCK_REMOTE_TEST_HOST"]
fn a_remote_and_a_shared_solve_agree_with_the_local_one() {
    use crate::analyze::solve::MachineState;
    use crate::app::settings::{Compute, Machine, MachineRef, Mode as ComputeMode};
    const CAL: f64 = 0.0762;
    let host = std::env::var("BOWSHOCK_REMOTE_TEST_HOST").expect("set BOWSHOCK_REMOTE_TEST_HOST (user@host)");
    let machine = Machine::parse(&host);
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let nodes = vec![0.5, 0.8, 1.1, 1.5, 2.0, 2.5];
    let cancel = Arc::new(AtomicBool::new(false));
    let run = |label: &str, compute: Compute| {
        let log = Arc::new(Mutex::new((Vec::<String>::new(), Vec::<usize>::new())));
        let tap = |log: Arc<Mutex<(Vec<String>, Vec<usize>)>>| {
            move |e: Event| {
                let mut log = log.lock().unwrap();
                match e {
                    Event::Machine { index, state: MachineState::Ready(d), .. } => log.0.push(format!("{index} ready on {d}")),
                    Event::Machine { index, state: MachineState::Skipped(w) | MachineState::Dropped(w), .. } => log.0.push(format!("{index} left: {w}")),
                    Event::Point(p) => log.1.push(p.machine),
                    _ => {}
                }
            }
        };
        let t0 = Instant::now();
        let drag_job =
            Job { plan: plan.clone(), altitude: 120.0, nodes: nodes.clone(), stage: 1, grid: CfdGrid::Coarse, compute: compute.clone(), cpu_only: false };
        let drag = solve(&drag_job, &cancel, tap(log.clone())).expect("the drag table solves");
        let drag_s = t0.elapsed().as_secs_f64();
        let lift_job = LiftJob { plan: plan.clone(), nodes: nodes.clone(), compute, cpu_only: false };
        let lift = solve_lift(&lift_job, &cancel, tap(log.clone())).expect("the 3D table solves");
        let lift_s = t0.elapsed().as_secs_f64() - drag_s;
        let log = log.lock().unwrap();
        let per = |m: usize| log.1.iter().filter(|&&p| p == m).count();
        println!("{label}: drag {drag_s:.1} s [{}], 3D {lift_s:.1} s [{}]", drag.backend, lift.backend);
        println!("  machines {:?}; points per machine (drag and 3D together) {:?}", log.0, (0..2).map(per).collect::<Vec<_>>());
        (drag, lift)
    };
    let remote = Compute { machines: vec![machine.clone()], mode: ComputeMode::Pick(MachineRef::Machine(machine.key())), ..Compute::default() };
    let shared = Compute { machines: vec![machine], mode: ComputeMode::Share, ..Compute::default() };
    let (local_drag, local_lift) = run("this computer", Compute::default());
    let (remote_drag, remote_lift) = run("remote only", remote);
    let (shared_drag, shared_lift) = run("shared", shared);
    assert!(remote_drag.backend.contains(" on "), "{}", remote_drag.backend);
    assert!(shared_drag.backend.contains("on this computer") || shared_drag.backend.contains(" on "), "{}", shared_drag.backend);
    for (name, drag, lift) in [("remote", &remote_drag, &remote_lift), ("shared", &shared_drag, &shared_lift)] {
        assert_eq!(local_drag.points.len(), drag.points.len(), "{name}");
        for (l, r) in local_drag.points.iter().zip(&drag.points) {
            let rel = (l.cd - r.cd).abs() / l.cd;
            println!("{name}: Mach {}: Cd local {:.5} {:.5} ({:.3} %)", l.mach, l.cd, r.cd, 100.0 * rel);
            assert!(l.mach == r.mach && rel < 0.005, "{name} Cd at Mach {}: {} vs {}", l.mach, l.cd, r.cd);
        }
        assert_eq!(local_lift.points.len(), lift.points.len(), "{name}");
        for (l, r) in local_lift.points.iter().zip(&lift.points) {
            let rel = (l.cn - r.cn).abs() / l.cn.abs();
            let cal = (l.x_cp - r.x_cp).abs() / CAL;
            println!(
                "{name}: Mach {} a {}: CN local {:.5} {:.5} ({:.3} %), CP {:.4} vs {:.4} m ({cal:.4} cal)",
                l.mach,
                l.alpha_deg,
                l.cn,
                r.cn,
                100.0 * rel,
                l.x_cp,
                r.x_cp
            );
            assert!(l.mach == r.mach && l.alpha_deg == r.alpha_deg, "{name}: points in Mach order");
            assert!(rel < 0.005, "{name} CN at Mach {} {} deg: {} vs {}", l.mach, l.alpha_deg, l.cn, r.cn);
            assert!(cal < 0.01, "{name} CP at Mach {} {} deg: {} vs {} m", l.mach, l.alpha_deg, l.x_cp, r.x_cp);
        }
    }
}
