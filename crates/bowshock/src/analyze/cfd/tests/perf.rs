//! Performance baselines and A/B studies of the solves as the app runs them: all `#[ignore]`d (they solve on the GPU), instrumentation
//! only. Their `PERF` lines go to the ledger (`perf/ledger.jsonl`, see `bowshock-validate`'s `perf_e2e`).

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use bowshock_model::CfdGrid;

use super::*;
use crate::units::UnitSystem;

/// Prints the per-point timing table of the runs in `tm` (see `bowshock_core::engine::gpu::PointTiming`) and the `PERF` line of
/// the end-to-end case `case` (`wall` seconds in total). Instrumentation only.
fn timing_report(case: &str, tm: &[solver::engine::gpu::PointTiming], wall: f64) {
    let perf = !case.starts_with("diag-");
    println!("dim mach alpha warm cells | build ctx upload reset | first_chunk rest(n,s) | dl(n,s) forces final | run | iters(stop/exec) stop");
    for t in tm {
        println!(
            "{} {:.2} {:.0} {} {} | {:.3} {:.3} {:.3} {:.3} | {:.3} ({},{:.3}) | ({},{:.3}) {:.3} {:.3} | {:.3} | {}/{} {}",
            t.dim,
            t.mach,
            t.alpha_deg,
            if t.warm { "warm" } else { "cold" },
            t.cells,
            t.build_s,
            t.ctx_s,
            t.upload_s,
            t.reset_s,
            t.first_chunk_s,
            t.chunks_rest,
            t.rest_chunks_s,
            t.downloads,
            t.download_s,
            t.forces_s,
            t.final_s,
            t.run_s,
            t.iters_stop,
            t.iters_exec,
            t.stop
        );
    }
    let sum = |f: fn(&solver::engine::gpu::PointTiming) -> f64| tm.iter().map(f).sum::<f64>();
    let iters: usize = tm.iter().map(|t| t.iters_exec).sum();
    println!(
        "TOTAL wall {wall:.2} s over {} runs ({} cold): build {:.2} ctx {:.2} upload {:.2} reset {:.2} first_chunks {:.2} rest_chunks {:.2} downloads {:.2} forces {:.2} final {:.2} run_sum {:.2}; {iters} iterations",
        tm.len(),
        tm.iter().filter(|t| !t.warm).count(),
        sum(|t| t.build_s),
        sum(|t| t.ctx_s),
        sum(|t| t.upload_s),
        sum(|t| t.reset_s),
        sum(|t| t.first_chunk_s),
        sum(|t| t.rest_chunks_s),
        sum(|t| t.download_s),
        sum(|t| t.forces_s),
        sum(|t| t.final_s),
        sum(|t| t.run_s),
    );
    let cells = tm.iter().map(|t| t.cells).max().unwrap_or(0);
    if perf {
        // The 3D solve runs in f32 on the GPU (`Analysis::run` at an angle of attack), the 2D drag table in df32.
        let precision = if case.starts_with("3d") { "f32" } else { "df32" };
        println!(
            "PERF {{\"case\":\"{case}\",\"precision\":\"{precision}\",\"cells\":{cells},\"iters\":{iters},\"us_per_iter\":{:.3},\"launches_per_iter\":null,\"gpu_us_per_iter\":null,\"wall_s\":{wall:.3}}}",
            1e6 * wall / iters.max(1) as f64
        );
    }
}

/// End to end: the example rocket's drag table as the app solves it (`solve`, 10 Mach numbers over [0.1, 3.0], altitude 120 m,
/// coarse grid, this computer alone), with the per-point timing breakdown. Prints a `PERF` line for the `gpuq` ledger.
#[test]
#[ignore = "perf baseline: solves the example's whole drag table on the GPU"]
fn perf_2d_table_example() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: nodes((0.1, 3.0)).unwrap(),
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
        stability: Vec::new(),
    };
    let _ = solver::engine::gpu::take_timings();
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), |_| ()).expect("the example solves");
    let wall = t0.elapsed().as_secs_f64();
    println!("{} points on {} in {:.1} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
    timing_report("2d-table-example", &solver::engine::gpu::take_timings(), wall);
}

/// End to end: the example rocket's 3D stability solve as the stability nodes of `solve` run it (the lift nodes of the example's Mach range
/// [0.1, 3.0], alpha 2 and 6 degrees, coarse grid, this computer alone), with the per-point timing breakdown and a `PERF` line.
#[test]
#[ignore = "perf baseline: solves the example's 3D stability table on the GPU"]
fn perf_3d_lift_example() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    // the angles alone: no drag nodes, so the time stays the 3D part's (the ledger's `3d-lift-example`)
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: Vec::new(),
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
        stability: lift_nodes((0.1, 3.0)).unwrap(),
    };
    let _ = solver::engine::gpu::take_timings();
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), |_| ()).expect("the example solves in 3D").lift.expect("with its 3D part");
    let wall = t0.elapsed().as_secs_f64();
    println!("{} points on {} in {:.1} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
    for p in &table.points {
        println!("M {:.2} alpha {:.0} CN {:.4} x_cp {:.4} {:?}", p.mach, p.alpha_deg, p.cn, p.x_cp, p.quality);
    }
    timing_report("3d-lift-example", &solver::engine::gpu::take_timings(), wall);
}

/// A/B of the grid shape of the app's table solve: `BOWSHOCK_GRID=own` (a grid sized for each point's Mach number, the old way)
/// or `shared` (one grid sized for the highest node): the example's drag table with a `RESULT` line per point (Cd parts and
/// iterations) and the timing breakdown.
#[test]
#[ignore = "A/B: solves the example's drag table twice-shaped on the GPU"]
fn perf_2d_table_grid_ab() {
    let shared = match std::env::var("BOWSHOCK_GRID").as_deref() {
        Ok("own") => false,
        Ok("shared") => true,
        other => panic!("BOWSHOCK_GRID must be own or shared, got {other:?}"),
    };
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: nodes((0.1, 3.0)).unwrap(),
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
        stability: Vec::new(),
    };
    let iters = Arc::new(Mutex::new(Vec::new()));
    let sink = iters.clone();
    let _ = solver::engine::gpu::take_timings();
    let t0 = Instant::now();
    let table = solve_sized(
        &job,
        &Arc::new(AtomicBool::new(false)),
        move |e| {
            if let Event::Point(p) = e {
                sink.lock().unwrap().push(p.iterations);
            }
        },
        shared,
    )
    .expect("the example solves");
    let wall = t0.elapsed().as_secs_f64();
    let iters = iters.lock().unwrap();
    for (p, n) in table.points.iter().zip(iters.iter()) {
        println!(
            "RESULT {{\"grid\":\"{}\",\"mach\":{},\"cd\":{:.6},\"friction\":{:.6},\"pressure\":{:.6},\"base\":{:.6},\"fins\":{:.6},\"other\":{:.6},\"quality\":\"{:?}\",\"iterations\":{n}}}",
            if shared { "shared" } else { "own" },
            p.mach,
            p.cd,
            p.friction,
            p.pressure,
            p.base,
            p.fins,
            p.other,
            p.quality
        );
    }
    timing_report(if shared { "2d-table-grid-shared" } else { "2d-table-grid-own" }, &solver::engine::gpu::take_timings(), wall);
}

/// Diagnostic: how the Cd of the app's points at M 0.1 and 1.0 depends on the grid shape (own / shared) and on the stop rule:
/// the app's rule, a shorter mean-stationary window, and fixed-length runs without any stop rule (Cd at several iteration counts
/// shows the size of the limit cycle). One `STUDY` line per run.
#[test]
#[ignore = "diagnostic: M 0.1 / 1.0 stop-rule and grid-shape study on the GPU"]
fn perf_2d_stop_rule_study() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let build = |shared: bool| {
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: 120.0 } })
            .drag_tolerance(DRAG_TOLERANCE)
            .resolution(solver::Resolution::Coarse)
            .roughness(plan.body_roughness)
            .backend(solver::Backend::Gpu);
        if shared {
            a = a.grid_mach(3.0);
        }
        if let Some(fins) = &plan.fins {
            a = a.fins(fins.clone());
        }
        a
    };
    // (label, mean_window, fixed iterations)
    let variants: [(&str, Option<usize>, Option<usize>); 7] = [
        ("app", None, None),
        ("window3000", Some(3000), None),
        ("window2000", Some(2000), None),
        ("fixed16000", None, Some(16000)),
        ("fixed20000", None, Some(20000)),
        ("fixed24000", None, Some(24000)),
        ("fixed28000", None, Some(28000)),
    ];
    for mach in [0.1, 1.0] {
        for shared in [false, true] {
            for (label, window, fixed) in variants {
                if mach > 0.5 && !(label == "app" || label == "fixed16000" || label == "fixed24000") {
                    continue;
                }
                let base = build(shared);
                let mut n = base.engine_cases(&[mach]).unwrap()[0].numerics;
                if let Some(w) = window {
                    n.mean_window = Some(w);
                }
                if let Some(f) = fixed {
                    (n.max_iter, n.min_iter) = (f, f);
                }
                let t0 = Instant::now();
                let polar = base.numerics(n).run(&[mach]).expect("the point solves");
                let p = &polar.points[0];
                println!(
                    "STUDY {{\"mach\":{mach},\"grid\":\"{}\",\"variant\":\"{label}\",\"iterations\":{},\"stop\":\"{:?}\",\"cd_solver\":{:.6},\"friction\":{:.6},\"pressure\":{:.6},\"base\":{:.6},\"wall_s\":{:.2}}}",
                    if shared { "shared" } else { "own" },
                    p.detail.iterations,
                    p.detail.stop_reason,
                    p.drag.forebody_pressure + p.drag.friction.unwrap_or(0.0) + p.drag.base,
                    p.drag.friction.unwrap_or(0.0),
                    p.drag.forebody_pressure,
                    p.drag.base,
                    t0.elapsed().as_secs_f64()
                );
            }
        }
    }
}

/// Diagnostic (no `PERF` line): the same 10 Mach numbers as one `Analysis::run` sweep (warm starts from point to point, one grid
/// for all points sized for the highest Mach), to show what the app's one-`Analysis`-per-Mach solve gives up.
#[test]
#[ignore = "diagnostic: the example's drag table as one warm-started sweep on the GPU"]
fn perf_2d_table_one_sweep() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let machs = nodes((0.1, 3.0)).unwrap();
    let mut analysis = solver::Analysis::new(&plan.body)
        .unwrap()
        .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: 120.0 } })
        .drag_tolerance(DRAG_TOLERANCE)
        .resolution(solver::Resolution::Coarse)
        .roughness(plan.body_roughness);
    if let Some(fins) = &plan.fins {
        analysis = analysis.fins(fins.clone());
    }
    let _ = solver::engine::gpu::take_timings();
    let t0 = Instant::now();
    let polar = analysis.run(&machs).expect("the sweep solves");
    let wall = t0.elapsed().as_secs_f64();
    for p in &polar.points {
        println!("M {:.2} Cd {:.4} {:?} {} iterations", p.mach, p.drag.total, p.quality, p.detail.iterations);
    }
    timing_report("diag-2d-one-sweep", &solver::engine::gpu::take_timings(), wall);
}
