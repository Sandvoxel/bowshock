//! Performance baselines and A/B studies of the solves as the app runs them: all `#[ignore]`d (they solve on the GPU), instrumentation
//! only. Their `PERF` lines go to the ledger (`perf/ledger.jsonl`, see `bowshock-validate`'s `perf_e2e`).

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use bowshock_model::CfdGrid;

use super::*;
use crate::units::UnitSystem;

/// Prints the per-point timing table of the runs in `tm` (see `bowshock_core::engine::timing::PointTiming`) and the `PERF` line of
/// the end-to-end case `case` (`wall` seconds in total). Instrumentation only.
fn timing_report(case: &str, tm: &[solver::engine::timing::PointTiming], wall: f64) {
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
    let sum = |f: fn(&solver::engine::timing::PointTiming) -> f64| tm.iter().map(f).sum::<f64>();
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
        power_on: Vec::new(),
    };
    let _ = solver::engine::timing::take_timings();
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), |_| ()).expect("the example solves");
    let wall = t0.elapsed().as_secs_f64();
    println!("{} points on {} in {:.1} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
    timing_report("2d-table-example", &solver::engine::timing::take_timings(), wall);
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
        power_on: Vec::new(),
    };
    let _ = solver::engine::timing::take_timings();
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), |_| ()).expect("the example solves in 3D").lift.expect("with its 3D part");
    let wall = t0.elapsed().as_secs_f64();
    println!("{} points on {} in {:.1} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
    for p in &table.points {
        println!("M {:.2} alpha {:.0} CN {:.4} x_cp {:.4} {:?}", p.mach, p.alpha_deg, p.cn, p.x_cp, p.quality);
    }
    timing_report("3d-lift-example", &solver::engine::timing::take_timings(), wall);
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
        power_on: Vec::new(),
    };
    let iters = Arc::new(Mutex::new(Vec::new()));
    let sink = iters.clone();
    let _ = solver::engine::timing::take_timings();
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
    timing_report(if shared { "2d-table-grid-shared" } else { "2d-table-grid-own" }, &solver::engine::timing::take_timings(), wall);
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
    let _ = solver::engine::timing::take_timings();
    let t0 = Instant::now();
    let polar = analysis.run(&machs).expect("the sweep solves");
    let wall = t0.elapsed().as_secs_f64();
    for p in &polar.points {
        println!("M {:.2} Cd {:.4} {:?} {} iterations", p.mach, p.drag.total, p.quality, p.detail.iterations);
    }
    timing_report("diag-2d-one-sweep", &solver::engine::timing::take_timings(), wall);
}

/// The example document without its fins (the 3D engine's finless wedge table), written to `BOWSHOCK_FINLESS_FIXTURE=<path>` for a drive script
/// (`BOWSHOCK_OPEN`): `BOWSHOCK_FINLESS_FIXTURE=/tmp/example-finless.bowshock cargo test -p bowshock -- --ignored the_finless_example --nocapture`.
#[test]
#[ignore = "writes the finless example document for the drive scripts"]
fn the_finless_example_document() {
    let (mut rocket, _) = example();
    let fins: Vec<_> = rocket.radials().iter().filter(|r| format!("{:?}", r.kind).contains("Fin")).map(|r| r.id).collect();
    assert!(!fins.is_empty(), "the example has fin sets");
    for id in fins {
        rocket.remove_radial(id);
    }
    let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
    assert!(plan(&rocket, &vehicle, UnitSystem::default()).unwrap().fins.is_none());
    let path = std::env::var_os("BOWSHOCK_FINLESS_FIXTURE").expect("set BOWSHOCK_FINLESS_FIXTURE=<path>");
    std::fs::write(path, rocket.to_bytes()).unwrap();
}

/// The half domain at incidence of the 3D engine (`Engine::Solver3`: the base-wake grid, the base solved) against today's sting-grid lift case (`Engine::Legacy`)
/// for the example rocket as the app solves its stability nodes (coarse grid, GPU): CN, `x_cp` and the axial force per Mach number and angle. The two differ
/// because the base is now solved instead of a sting and a mild fin-grid change; reported, not gated. `cargo nextest run --release -p bowshock half_domain_against --run-ignored only --no-capture`
/// through `tools/gpuq/gpuq`.
#[test]
#[ignore = "solves the example's stability nodes twice on the GPU"]
fn half_domain_against_the_sting_grid() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let nodes = lift_nodes((0.1, 3.0)).unwrap();
    let conditions: Vec<solver::Condition> = nodes.iter().flat_map(|&m| LIFT_ALPHAS.iter().map(move |&a| solver::Condition::new(m, a))).collect();
    let run = |engine: solver::Engine| {
        let mut a = solver::Analysis::new(&plan.body).unwrap().engine(engine).resolution(solver::Resolution::Coarse).backend(solver::Backend::Gpu);
        if let Some(f) = &plan.fins {
            a = a.fins(f.clone());
        }
        let t = Instant::now();
        let polar = a.run(&conditions).expect("the stability nodes solve");
        println!("{engine:?}: {:.1} s on {:?}", t.elapsed().as_secs_f64(), polar.backend);
        polar
    };
    let (old, new) = (run(solver::Engine::Legacy), run(solver::Engine::Solver3));
    let cal = plan.body.nose.diameter;
    println!("| Mach | alpha | CN sting | CN base | dCN % | x_cp sting m | x_cp base m | d x_cp mm | d x_cp cal | CA sting | CA base | its sting | its base | quality base |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (o, n) in old.points.iter().zip(&new.points) {
        let (fo, fn_) = (o.forces.unwrap(), n.forces.unwrap());
        println!(
            "| {:.2} | {:.0} | {:.4} | {:.4} | {:+.1} | {:.4} | {:.4} | {:+.1} | {:+.3} | {:.4} | {:.4} | {} | {} | {:?} |",
            o.mach,
            o.alpha_deg,
            fo.cn,
            fn_.cn,
            100.0 * (fn_.cn - fo.cn) / fo.cn,
            fo.x_cp,
            fn_.x_cp,
            1000.0 * (fn_.x_cp - fo.x_cp),
            (fn_.x_cp - fo.x_cp) / cal,
            fo.ca,
            fn_.ca,
            o.detail.iterations,
            n.detail.iterations,
            n.quality
        );
    }
}

/// Where the time of the 3D engine goes, through the app path: the FINNED example rocket as the app solves it under `BOWSHOCK_ENGINE=3d`: 10 sector points
/// at zero incidence, then 8 Mach numbers at 2 and 6 degrees on the half domain, one `run` per node (`solve`). Prints one line per solved point (the runner's
/// `PointTiming` joined with the facade's `FacadeTiming`), the sums by kind and the whole solve, and a `PERF` line.
/// `tools/gpuq/gpuq submit --host local --name engine3d-timing -- env BOWSHOCK_ENGINE=3d cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture`.
#[test]
#[ignore = "BOWSHOCK_ENGINE=3d: solves the finned example's table and angles on the GPU"]
fn engine3d_finned_timing() {
    assert_eq!(solver::Engine::from_env(), solver::Engine::Solver3, "run with BOWSHOCK_ENGINE=3d");
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert!(plan.fins.is_some());
    let shared = std::env::var("BOWSHOCK_SHARED_GRID").is_ok();
    // the power-on part of the app's table (the motor's jet at the nodes of `POWER_ON_NODES`) is part of the full solve; BOWSHOCK_NO_POWER_ON=1 leaves it out (the baseline)
    let jet = plan.jet();
    let power_on =
        if plan.motor.is_some() && std::env::var_os("BOWSHOCK_NO_POWER_ON").is_none() { power_on_nodes(&nodes((0.1, 3.0)).unwrap()) } else { Vec::new() };
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: nodes((0.1, 3.0)).unwrap(),
        stage: 1,
        grid: if std::env::var("BOWSHOCK_GRID").is_ok_and(|g| g == "draft") { CfdGrid::Draft } else { CfdGrid::Coarse },
        compute: Default::default(),
        cpu_only: false,
        stability: lift_nodes((0.1, 3.0)).unwrap(),
        power_on,
    };
    let _ = solver::engine::timing::take_timings();
    let _ = solver::take_facade_timings();
    let t0 = Instant::now();
    let table = solve_sized(&job, &Arc::new(AtomicBool::new(false)), |_| (), shared).expect("the example solves");
    let wall = t0.elapsed().as_secs_f64();
    let runs = solver::engine::timing::take_timings();
    let facade = solver::take_facade_timings();
    println!(
        "{} points + {} angle points on {}; {:.1} s wall (app solve {:.1} s); notes {:?}",
        table.points.len(),
        table.lift.as_ref().map_or(0, |l| l.points.len()),
        table.backend,
        wall,
        table.solve_seconds,
        table.notes
    );
    for p in &table.points {
        println!(
            "DRAG M {:.2} cd {:.5} friction {:.5} pressure {:.5} base {:.5} fins {:.5} other {:.5} {:?}",
            p.mach, p.cd, p.friction, p.pressure, p.base, p.fins, p.other, p.quality
        );
    }
    if let Some(l) = &table.lift {
        for p in &l.points {
            println!("LIFT M {:.2} alpha {:.0} CN {:.4} x_cp {:.4} cd {:?} {:?}", p.mach, p.alpha_deg, p.cn, p.x_cp, p.cd, p.quality);
        }
    }
    if let Some(on) = &table.power_on {
        println!("JET {:?}", on.jet);
        for p in &on.points {
            println!("POWER M {:.2} friction x{:.4} pressure x{:.4} base x{:.4} {:?}", p.mach, p.friction, p.pressure, p.base, p.quality);
        }
        super::power_on::print_apogee_effect("example", &rocket, &table, jet.as_ref());
        // the same ratios on the Draft grid (BOWSHOCK_NO_DRAFT_RATIOS=1 leaves it out): what a cheaper power-on solve would give
        if std::env::var_os("BOWSHOCK_NO_DRAFT_RATIOS").is_none() {
            let plan = super::plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
            super::power_on::solve_ratios("example", &plan, &table, CfdGrid::Draft);
        }
        // the long-burn case (BOWSHOCK_NO_LONG_BURN=1 leaves it out): the same rocket with a 6.5 s motor, its jet at the same nodes, on the example's table
        if std::env::var_os("BOWSHOCK_NO_LONG_BURN").is_none() {
            super::power_on::long_burn(&rocket, &table, "RATT", "I90L", table.grid);
        }
    }
    println!("| dim | Mach | alpha | cells | iters (stop/exec) | stop | build | ctx | upload | reset | first chunk | rest chunks n | rest s | ms/iter | monitor forces | final |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    // one row per run of a solver (a retried point has two); the facade's own costs are per point and are summed by kind below
    let mut sums = [0.0f64; 8];
    for r in &runs {
        let per_iter = if r.chunks_rest > 0 { 1000.0 * r.rest_chunks_s / (r.chunks_rest * 50) as f64 } else { f64::NAN };
        println!(
            "| {}D | {:.2} | {:.0} | {} | {}/{} | {} | {:.3} | {:.3} | {:.3} | {:.3} | {:.3} | {} | {:.3} | {:.2} | {:.3} | {:.3} |",
            r.dim,
            r.mach,
            r.alpha_deg,
            r.cells,
            r.iters_stop,
            r.iters_exec,
            r.stop,
            r.build_s,
            r.ctx_s,
            r.upload_s,
            r.reset_s,
            r.first_chunk_s,
            r.chunks_rest,
            r.rest_chunks_s,
            per_iter,
            r.forces_s + r.download_s,
            r.final_s,
        );
        for (k, v) in
            [r.build_s, r.ctx_s, r.upload_s, r.reset_s, r.first_chunk_s, r.rest_chunks_s, r.forces_s + r.download_s, r.final_s].into_iter().enumerate()
        {
            sums[k] += v;
        }
    }
    println!("| runs | build | ctx | upload | reset | first chunk | rest chunks | monitor forces | final |");
    println!("| {} runs | {} |", runs.len(), sums.iter().map(|x| format!("{x:.2}")).collect::<Vec<_>>().join(" | "));
    let by = |what: &str, f: &dyn Fn(&solver::FacadeTiming) -> f64| facade.iter().filter(|t| t.what == what).map(f).sum::<f64>();
    let (case_s, post_s, host_s, field_s, drop_s) =
        (by("point", &|t| t.case_s), by("point", &|t| t.post_s), by("point", &|t| t.host_s), by("point", &|t| t.field_s), by("drop", &|t| t.drop_s));
    println!(
        "REUSED {} of {} cold points ran on a kept GPU solver",
        facade.iter().filter(|t| t.what == "point" && t.reused).count(),
        facade.iter().filter(|t| t.what == "point").count()
    );
    println!("| facade: {} points | case {case_s:.3} | post (device reduction + state download) {post_s:.3} | host (guards, wall samples) {host_s:.3} | Field3 {field_s:.3} | teardown {drop_s:.3} |", facade.iter().filter(|t| t.what == "point").count());
    let iterations: f64 = runs.iter().map(|r| r.iters_exec as f64).sum();
    let capped: Vec<String> =
        runs.iter().filter(|r| r.iters_exec >= 7500).map(|r| format!("{:.2}/{:.0} ({} it, {:.1} s)", r.mach, r.alpha_deg, r.iters_exec, r.run_s)).collect();
    let capped_s: f64 = runs.iter().filter(|r| r.iters_exec >= 7500).map(|r| r.run_s).sum();
    let run_total: f64 = runs.iter().map(|r| r.run_s).sum();
    let fixed = sums[0] + sums[1] + sums[2] + sums[3] + sums[4] + case_s + post_s + host_s + field_s + drop_s;
    println!("SUM {} runs, {iterations} iterations: iterating {run_total:.1} s of {wall:.1} s wall; fixed costs per point (build, context, upload, reset, first chunk, facade) {fixed:.1} s; points at the iteration cap: {capped:?} = {capped_s:.1} s", runs.len());
    let all = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, iterations];
    println!("PERF {}", serde_json_perf(wall, all[13]));
}

fn serde_json_perf(wall: f64, iters: f64) -> String {
    format!(
        "{{\"case\":\"3d-engine-finned-example\",\"precision\":\"f32\",\"cells\":null,\"iters\":{iters},\"us_per_iter\":{:.3},\"wall_s\":{wall:.3}}}",
        1e6 * wall / iters
    )
}

/// A/B of warm starts across Mach numbers on the 3D engine: the finned example's zero-incidence sector table (10 Mach numbers) cold per point against
/// `warm_start(true)` (ascending, each point from the neighbouring Mach number's field): iterations, Cd and wall per point, and the largest change of Cd.
/// The legacy 2D path found warm starts slower; this measures the 3D engine. `BOWSHOCK_WARM_HALF=1` does the same for the half domain (8 Mach numbers at 2 and 6 degrees).
#[test]
#[ignore = "A/B on the GPU: solves the finned example's sector table twice"]
fn engine3d_warm_start_ab() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let half = std::env::var("BOWSHOCK_WARM_HALF").is_ok();
    let conditions: Vec<solver::Condition> = if half {
        lift_nodes((0.1, 3.0)).unwrap().iter().flat_map(|&m| LIFT_ALPHAS.iter().map(move |&a| solver::Condition::new(m, a))).collect()
    } else {
        nodes((0.1, 3.0)).unwrap().into_iter().map(solver::Condition::from).collect()
    };
    let run = |warm: bool| {
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .engine(solver::Engine::Solver3)
            .resolution(solver::Resolution::Coarse)
            .backend(solver::Backend::Gpu)
            .warm_start(warm);
        if let Some(f) = &plan.fins {
            a = a.fins(f.clone());
        }
        let t = Instant::now();
        let polar = a.run(&conditions).expect("the points solve");
        (polar, t.elapsed().as_secs_f64())
    };
    let ((cold, t_cold), (warm, t_warm)) = (run(false), run(true));
    println!("cold {t_cold:.1} s, warm {t_warm:.1} s");
    println!("| Mach | alpha | its cold | its warm | stop cold | stop warm | value cold | value warm | change % |");
    println!("|---|---|---|---|---|---|---|---|---|");
    let (mut worst, mut its_c, mut its_w) = (0.0f64, 0usize, 0usize);
    for (c, w) in cold.points.iter().zip(&warm.points) {
        let value = |p: &solver::PolarPoint| p.forces.map_or(p.drag.total, |f| f.cn);
        let rel = 100.0 * (value(w) - value(c)) / value(c);
        worst = worst.max(rel.abs());
        its_c += c.detail.iterations;
        its_w += w.detail.iterations;
        println!(
            "| {:.2} | {:.0} | {} | {} | {:?} | {:?} | {:.5} | {:.5} | {:+.3} |",
            c.mach,
            c.alpha_deg,
            c.detail.iterations,
            w.detail.iterations,
            c.detail.stop_reason,
            w.detail.stop_reason,
            value(c),
            value(w),
            rel
        );
    }
    println!("WARM total iterations cold {its_c} warm {its_w}; worst change {worst:.3} % ({})", if half { "CN" } else { "Cd" });
}

/// What oscillates at the points the finless half domain does not settle (see the table of `half_domain_against_the_sting_grid`): the finless example body on
/// the base-wake half domain at M 0.5 and 0.8 at 2 and 6 degrees, M 0.95 at 2 degrees, with M 1.1 and 1.3 as settled controls. Runs 3000 iterations cold, then
/// 4000 more in chunks of 10 sampling CN, CA, the base axial force, the nose and afterbody CN from the device reductions: per series the mean, the slope over
/// the window (relative drift), the peak-to-peak and the dominant period (iterations) of the detrended samples.
#[test]
#[ignore = "GPU: eight cases of 7000 iterations on the half domain"]
fn engine3d_subsonic_wake_history() {
    use std::sync::Mutex;
    static CASE: Mutex<Option<solver::engine::case3::Case3>> = Mutex::new(None);
    fn capture(c: &mut solver::engine::case3::Case3) {
        *CASE.lock().unwrap() = Some(c.clone());
    }
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let analysis = || {
        solver::Analysis::new(&plan.body)
            .unwrap()
            .engine(solver::Engine::Solver3)
            .resolution(solver::Resolution::Coarse)
            .case3_hook(capture)
            .max_iterations(1)
            .backend(solver::Backend::Cpu)
    };
    // dominant period of a detrended series sampled every `dt` iterations: the largest DFT bin below Nyquist, (period, amplitude)
    let spectrum = |x: &[f64], dt: f64| -> (f64, f64) {
        let n = x.len();
        let mean = x.iter().sum::<f64>() / n as f64;
        let (mut best, mut at) = (0.0, 0usize);
        for k in 1..n / 2 {
            let (mut re, mut im) = (0.0, 0.0);
            for (i, v) in x.iter().enumerate() {
                let ph = std::f64::consts::TAU * (k * i) as f64 / n as f64;
                re += (v - mean) * ph.cos();
                im += (v - mean) * ph.sin();
            }
            let amp = 2.0 * re.hypot(im) / n as f64;
            if amp > best {
                (best, at) = (amp, k);
            }
        }
        (n as f64 * dt / at.max(1) as f64, best)
    };
    println!("| Mach | alpha | series | mean | drift over window % | p2p % of mean | dominant period (its) | amp % of mean |");
    println!("|---|---|---|---|---|---|---|---|");
    for (mach, alpha) in [(0.5, 2.0), (0.5, 6.0), (0.8, 2.0), (0.8, 6.0), (0.95, 2.0), (1.1, 6.0), (1.3, 2.0)] {
        let _ = analysis().run(&[solver::Condition::new(mach, alpha)]);
        let mut case = CASE.lock().unwrap().take().expect("the hook saw the case");
        case.numerics.max_iter = 3000;
        case.numerics.rel_tol = 0.0;
        case.snapshots = None;
        let mut r = solver::engine::gpu3::GpuRunner3::new(&case, solver::Precision::F32).expect("a GPU");
        let first = r.run_on(&case, false).expect("the first 3000 iterations");
        let chunk = 10;
        let mut series: [Vec<f64>; 5] = Default::default();
        let mut xcp: Vec<f64> = Vec::new();
        for _ in 0..400 {
            r.gpu.run(chunk).unwrap();
            let f = r.lift_forces(&case).unwrap();
            let b = r.breakdown(&case).unwrap();
            let d = r.drag_parts(&case).unwrap();
            for (s, v) in series.iter_mut().zip([f.cn, f.ca, d.base, b.nose.cn, b.afterbody.cn]) {
                s.push(v);
            }
            xcp.push(f.x_cp);
        }
        // the raw histories (iteration, CA, CA of the base, CN, x_cp in nose diameters), every 10 iterations from 3010 on: for the stop rule of the 3D engine
        std::fs::create_dir_all("out/wake-csv").unwrap();
        let rows: String =
            (0..xcp.len()).map(|i| format!("{},{:.8},{:.8},{:.8},{:.6}\n", 3010 + 10 * i, series[1][i], series[2][i], series[0][i], xcp[i])).collect();
        std::fs::write(format!("out/wake-csv/M{mach}_a{alpha:.0}.csv"), format!("it,ca,ca_base,cn,x_cp\n{rows}")).unwrap();
        println!("INFO M {mach} alpha {alpha}: first 3000 iterations stop {:?}, {:.2} orders", first.stop_reason, first.drop_orders);
        // an offline stop rule on the CN history sampled every 50 iterations (the first 3000 from the run, then every fifth sample of the series): the first
        // iteration at which two successive windows of 5 samples (250 iterations) have means within `tol` of each other, and what that mean is against the mean
        // over iterations 3000 to 7000
        let mut hist: Vec<(usize, f64)> = first.cn_history.clone();
        hist.extend(series[0].iter().enumerate().filter(|(i, _)| i % 5 == 4).map(|(i, v)| (3000 + 10 * (i + 1), *v)));
        let late = series[0].iter().sum::<f64>() / series[0].len() as f64;
        for tol in [0.002, 0.005, 0.01, 0.02] {
            let win = |end: usize| hist[end - 5..end].iter().map(|h| h.1).sum::<f64>() / 5.0;
            let fired = (15..=hist.len()).find(|&e| {
                let (m0, m1, m2) = (win(e - 10), win(e - 5), win(e));
                (m1 - m0).abs() < tol * m1.abs() && (m2 - m1).abs() < tol * m2.abs()
            });
            match fired {
                Some(e) => println!(
                    "RULE M {mach} alpha {alpha}: mean-stationary within {:.1} % fires at iteration {}, window mean {:.5} vs late mean {late:.5} ({:+.2} %)",
                    100.0 * tol,
                    hist[e - 1].0,
                    win(e),
                    100.0 * (win(e) - late) / late
                ),
                None => println!(
                    "RULE M {mach} alpha {alpha}: mean-stationary within {:.1} % never fires by iteration {}",
                    100.0 * tol,
                    hist.last().map_or(0, |h| h.0)
                ),
            }
        }
        for (name, x) in ["CN", "CA", "CA base", "CN nose", "CN aft"].iter().zip(&series) {
            let n = x.len() as f64;
            let mean = x.iter().sum::<f64>() / n;
            let (lo, hi) = x.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| (l.min(*v), h.max(*v)));
            // least-squares slope over the window, as the change across it
            let tbar = (n - 1.0) / 2.0;
            let slope = x.iter().enumerate().map(|(i, v)| (i as f64 - tbar) * (v - mean)).sum::<f64>()
                / x.iter().enumerate().map(|(i, _)| (i as f64 - tbar).powi(2)).sum::<f64>();
            let (period, amp) = spectrum(x, chunk as f64);
            println!(
                "| {mach} | {alpha:.0} | {name} | {mean:.5} | {:+.2} | {:.2} | {period:.0} | {:.2} |",
                100.0 * slope * (n - 1.0) / mean.abs().max(1e-12),
                100.0 * (hi - lo) / mean.abs().max(1e-12),
                100.0 * amp / mean.abs().max(1e-12)
            );
        }
    }
}

/// The finned example's half domain at Mach 3.0 and 6 degrees on the GPU, which went non-finite in single precision in the table runs: prints the point and its quality.
#[test]
#[ignore = "GPU: one half-domain point, with the precision retry"]
fn engine3d_mach3_6deg_point() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let mut a = solver::Analysis::new(&plan.body).unwrap().engine(solver::Engine::Solver3).resolution(solver::Resolution::Coarse).backend(solver::Backend::Gpu);
    if let Some(f) = &plan.fins {
        a = a.fins(f.clone());
    }
    let t = Instant::now();
    let p = a.run(&[solver::Condition::new(3.0, 6.0)]).expect("solves").points.remove(0);
    let f = p.forces.unwrap();
    println!(
        "M3 6deg: CN {:.4} CA {:.4} x_cp {:.4} m, {} iterations, {:?} {:?}, {:.1} s (CPU f64 reference: CN 0.9102 CA 0.3062 on the ANF case)",
        f.cn,
        f.ca,
        f.x_cp,
        p.detail.iterations,
        p.detail.stop_reason,
        p.quality,
        t.elapsed().as_secs_f64()
    );
}

/// The host-side costs of a 3D point that do not need a device: building the grid of the finned example (sector and half domain; cache miss, then hit), building the CPU
/// solver from it, the wall samples and the `Field3` of a free-stream state. CPU only.
#[test]
#[ignore = "CPU: builds the finned example's 3D grids"]
fn engine3d_host_costs() {
    use solver::engine::case3::{Case3, Tier};
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let a = solver::Analysis::new(&plan.body).unwrap().engine(solver::Engine::Solver3).resolution(solver::Resolution::Coarse).fins(plan.fins.clone().unwrap());
    static CASE: Mutex<Vec<Case3>> = Mutex::new(Vec::new());
    fn capture(c: &mut Case3) {
        CASE.lock().unwrap().push(c.clone());
    }
    let a = a.case3_hook(capture).max_iterations(1).backend(solver::Backend::Cpu);
    let _ = a.run(&[solver::Condition::from(2.0), solver::Condition::new(2.0, 6.0)]);
    let cases = std::mem::take(&mut *CASE.lock().unwrap());
    let _ = Tier::Euler;
    for case in &cases {
        let t = Instant::now();
        let s = case.build_solver_cached().expect("grid");
        let miss = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let _ = case.build_solver_cached().expect("grid");
        let hit = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let samples = solver::engine::run3::wall_samples(&s.grid, &s.u);
        let ws = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let f = solver::engine::field3::field3_of(case, &s.grid, &s.u, 100, 1.0);
        let fs = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let d = solver::engine::run3::drag_parts(case, &s.grid, &s.u, s.cfg.freestream[4]);
        let dp = t.elapsed().as_secs_f64();
        println!(
            "HOST {:?}: {}x{}x{} = {} cells; grid {miss:.3} s then {hit:.3} s from the cache (incl. the solver tables); wall_samples {ws:.3} s ({} faces); Field3 {fs:.3} s ({} cells); drag_parts {dp:.3} s ({:.3})",
            case.grid.domain,
            s.grid.ni,
            s.grid.nj,
            s.grid.nk,
            s.grid.ni * s.grid.nj * s.grid.nk,
            samples.len(),
            f.cells(),
            d.total
        );
    }
}

/// Cold single points of the finned example's half domain (the points that diverged in the table runs), each its own `run` call, capped at 1000 iterations:
/// the stop reason, iterations and quality: the same test at two commits tells whether the divergence came with a change of code.
#[test]
#[ignore = "GPU: five cold half-domain points"]
fn engine3d_probe_cold_half_points() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    for (mach, alpha) in [(0.5, 2.0), (0.8, 2.0), (0.8, 6.0), (3.0, 2.0), (3.0, 6.0)] {
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .engine(solver::Engine::Solver3)
            .resolution(solver::Resolution::Coarse)
            .backend(solver::Backend::Gpu)
            .max_iterations(1000);
        if let Some(f) = &plan.fins {
            a = a.fins(f.clone());
        }
        let p = a.run(&[solver::Condition::new(mach, alpha)]).expect("solves").points.remove(0);
        println!("PROBE M {mach} alpha {alpha}: {} iterations, {:?}, {:?}", p.detail.iterations, p.detail.stop_reason, p.quality);
    }
}

/// The finned example's drag table with the drag split the app's table does not keep, per Mach number: body friction, forebody pressure, base, fin friction, fin pressure
/// (the facade's `Drag`), the point's wall time, iterations and ms/iter, y+, from the analysis the app builds for its drag nodes (`node_analysis`). Works for both
/// engines (`BOWSHOCK_ENGINE=3d` or unset, the legacy 2D RANS with the empirical fins: its split has no fin friction).
#[test]
#[ignore = "solves the finned example's table on the GPU"]
fn finned_drag_split() {
    let (rocket, vehicle) = example();
    let mut plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    // BOWSHOCK_SMOOTH=1: the walls solved smooth (the body and the fins), to compare with a direct engine run that has no roughness
    if std::env::var_os("BOWSHOCK_SMOOTH").is_some() {
        plan.body_roughness = 0.0;
        if let Some(f) = plan.fins.as_mut() {
            f.roughness = 0.0;
        }
    }
    // BOWSHOCK_FINLESS=1: the body alone (the axisymmetric wedge), for the jet's effect on the wedge against the sector's
    if std::env::var_os("BOWSHOCK_FINLESS").is_some() {
        plan.fins = None;
    }
    println!("ROUGHNESS body {:.1} um, fins {:.1} um", plan.body_roughness * 1e6, plan.fins.as_ref().map_or(0.0, |f| f.roughness * 1e6));
    // BOWSHOCK_MACHS=1.0,1.1 picks the Mach numbers (default: the table's ten), BOWSHOCK_GRID=draft the quick grid
    let machs: Vec<f64> =
        std::env::var("BOWSHOCK_MACHS").map(|v| v.split(',').map(|m| m.trim().parse().unwrap()).collect()).unwrap_or_else(|_| nodes((0.1, 3.0)).unwrap());
    let grid = if std::env::var("BOWSHOCK_GRID").is_ok_and(|g| g == "draft") { CfdGrid::Draft } else { CfdGrid::Coarse };
    println!("GRID {grid:?}");
    // BOWSHOCK_POWER=1: the nodes with the jet of the example's motor on (`NodeKind::PowerOn`, `plan.motor`), the power-on part of the app's table
    let kind = if std::env::var_os("BOWSHOCK_POWER").is_some() { NodeKind::PowerOn } else { NodeKind::Drag };
    println!("KIND {} JET {:?}", if kind == NodeKind::PowerOn { "power on" } else { "power off" }, plan.motor);
    let grid_mach = machs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: machs.clone(),
        stage: 1,
        grid,
        compute: Default::default(),
        cpu_only: false,
        stability: Vec::new(),
        power_on: Vec::new(),
    };
    let session = Session::Local { backend: solver::Backend::Gpu };
    println!("ENGINE {:?}", solver::Engine::from_env());
    println!(
        "| Mach | CD | body friction | forebody pressure | base | fin friction | fin pressure | iterations | wall s | ms/iter | y+ max | Cp base | stop |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let t_all = Instant::now();
    for &mach in &machs {
        let analysis = node_analysis(&job, kind, &session, 120.0, grid_mach, solves_3d()).unwrap_or_else(|_| panic!("the node analysis"));
        let _ = solver::engine::timing::take_timings();
        let t = Instant::now();
        let polar = analysis.run(&[mach]).expect("the point solves");
        let wall = t.elapsed().as_secs_f64();
        // every run of the point (a retry after a NonFinite shows as a second row)
        for r in solver::engine::timing::take_timings() {
            println!("RUN M {mach:.2}: {} cells, {} its, stop {}, {:.1} s", r.cells, r.iters_exec, r.stop, r.run_s);
        }
        let (p, d) = (&polar.points[0], &polar.points[0].drag);
        let (ff, fp) = d.fins.map_or((f64::NAN, f64::NAN), |f| (f.friction, f.pressure));
        println!(
            "| {mach:.2} | {:.5} | {:.5} | {:.5} | {:.5} | {ff:.5} | {fp:.5} | {} | {wall:.1} | {:.2} | {:.2} | {:.4} | {} |",
            d.total,
            d.friction.unwrap_or(f64::NAN),
            d.forebody_pressure,
            d.base,
            p.detail.iterations,
            1000.0 * wall / p.detail.iterations as f64,
            p.detail.y_plus_max,
            p.detail.cp_base,
            p.detail.stop_reason
        );
    }
    println!("TOTAL {:.1} s for {} points", t_all.elapsed().as_secs_f64(), machs.len());
}

/// One subsonic point of the finned example through the app's path (`solve_sized` with one node), to see what a point that diverges looks like to the user: how long it
/// takes, the error or the note, the runs behind it (a retry with the CFL halved is a second `RUN` row) and the facade's own verdict on the point.
#[test]
#[ignore = "solves one subsonic point of the finned example on the GPU"]
fn finned_subsonic_failure_path() {
    let mach: f64 = std::env::var("BOWSHOCK_MACH").ok().and_then(|m| m.parse().ok()).unwrap_or(0.5);
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let job = Job {
        plan,
        altitude: 120.0,
        nodes: vec![mach],
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
        stability: Vec::new(),
        power_on: Vec::new(),
    };
    let _ = solver::engine::timing::take_timings();
    let t = Instant::now();
    let table =
        solve_sized(&job, &Arc::new(AtomicBool::new(false)), |e| println!("EVENT {}", format!("{e:?}").chars().take(160).collect::<String>()), solves_3d());
    println!("APP RESULT after {:.1} s: {:?}", t.elapsed().as_secs_f64(), table.as_ref().map(|t| (t.points.len(), t.notes.clone())));
    for r in solver::engine::timing::take_timings() {
        println!("RUN M {:.2}: {} cells, {} its (stop {}), {:.1} s, {}", r.mach, r.cells, r.iters_exec, r.iters_stop, r.run_s, r.stop);
    }
}

/// The fins the app's plan gives the example document, for comparing with another harness's helper.
#[test]
#[ignore = "prints the example's solved fins"]
fn example_fins_dump() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let d = plan.body.nose.diameter;
    println!("NOSE D {d}  REF D {}", vehicle.reference_diameter);
    println!("FINS {:?}", plan.fins);
    if let Some(f) = &plan.fins {
        println!(
            "FINS in calibres: count {} root {:.4} tip {:.4} span {:.4} sweep {:.4} t {:.5} te_to_base {:.4} roughness {} profile {:?}",
            f.count,
            f.root_chord / d,
            f.tip_chord / d,
            f.span / d,
            f.sweep / d,
            f.thickness / d,
            f.root_te_to_base / d,
            f.roughness,
            f.profile
        );
    }
}

/// The stop rule of an angle-of-attack point of the stability solve, as the app solves it (the example rocket's half domain at 2 degrees, the Lift tier on the Coarse grid, one cold
/// `run` per Mach number): stop reason, iterations, wall seconds, CN and `x_cp` (calibres from the nose). `BOWSHOCK_INC_REF=1` runs the reference instead: the range rule off, the mean rule at
/// 5e-4 over windows of 2000, 24 000 iterations at most. `BOWSHOCK_MACHS=0.5,1.1,2.3` picks the Mach numbers.
#[test]
#[ignore = "BOWSHOCK_ENGINE=3d: three half-domain points on the GPU"]
fn engine3d_incidence_stop_rule() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let machs: Vec<f64> =
        std::env::var("BOWSHOCK_MACHS").map(|v| v.split(',').map(|m| m.trim().parse().unwrap()).collect()).unwrap_or_else(|_| vec![0.5, 1.1, 2.3]);
    let reference = std::env::var_os("BOWSHOCK_INC_REF").is_some();
    let cal = plan.body.nose.diameter;
    println!("ENGINE {:?} reference {reference}", solver::Engine::from_env());
    for mach in machs {
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .resolution(solver::Resolution::Coarse)
            .backend(solver::Backend::Gpu)
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: 0.0 } })
            .roughness(plan.body_roughness);
        if let Some(f) = &plan.fins {
            a = a.fins(f.clone()).fin_outline(plan.outline.clone());
        }
        if reference {
            a = a.case3_hook(|c| {
                c.numerics.rel_tol = 0.0;
                c.numerics.incidence_tol = 5e-4;
                (c.numerics.incidence_window, c.numerics.incidence_from) = (2000, 2000);
                c.numerics.max_iter = 24_000;
            });
        } else if std::env::var_os("BOWSHOCK_INC_WIN").is_some() {
            // a shorter mean window at incidence (BOWSHOCK_INC_WIN, BOWSHOCK_INC_FROM), the range rule as it is
            a = a.case3_hook(|c| {
                c.numerics.incidence_window = std::env::var("BOWSHOCK_INC_WIN").unwrap().parse().unwrap();
                c.numerics.incidence_from = std::env::var("BOWSHOCK_INC_FROM").unwrap().parse().unwrap();
            });
        }
        let t = Instant::now();
        let p = a.run(&[solver::Condition::new(mach, LIFT_ALPHAS[0])]).expect("solves").points.remove(0);
        let wall = t.elapsed().as_secs_f64();
        let f = p.forces.expect("forces");
        println!(
            "INC M {mach} its {} stop {:?} wall {wall:.1} s CN {:.5} x_cp {:.4} m {:.4} cal quality {:?}",
            p.detail.iterations,
            p.detail.stop_reason,
            f.cn,
            f.x_cp,
            f.x_cp / cal,
            p.quality
        );
    }
}
