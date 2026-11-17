//! `cfl_scan`: iterations to convergence of the wedge on the GPU against `cfl_max` and the line solve (stage S2: the pressure-source block of the j factor,
//! `Solver3::is_wedge`, is what makes `cfl_max` above about 1 possible).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example cfl_scan -- [--mach 2.0,0.9] [--cfl 5,10,20,50] [--solve block,diag] [--precision f32|df32] [--cpu]
//! ```
//!
//! The Euler base-wake grid of the example rocket (`engine_case(example_body, mach, false, Coarse)`: the grid of the Euler table), one azimuthal cell, the
//! stop rule of the facade (`Numerics3::auto`: residual drop or stationary `ca`). One line per (Mach, line solve, `cfl_max`).

use bowshock_cfd::gpu::Precision;
use bowshock_cfd::implicit::LineSolve3;
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::{engine_case, wedge_case3};
use std::time::Instant;

fn example_body() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] }
}

fn list<T: std::str::FromStr>(v: Option<String>, default: &str) -> Vec<T> {
    v.as_deref().unwrap_or(default).split(',').map(|x| x.trim().parse().unwrap_or_else(|_| panic!("cannot parse `{x}`"))).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let machs: Vec<f64> = list(value("--mach"), "2.0,0.9");
    let cfls: Vec<f64> = list(value("--cfl"), "5,10,20,50");
    let solves: Vec<String> = list(value("--solve"), "block,diag");
    let cpu = args.iter().any(|a| a == "--cpu");
    let prec = if value("--precision").as_deref() == Some("df32") { Precision::Extended } else { Precision::F32 };
    println!("| Mach | grid | line solve | cfl_max | iterations | stop | drop orders | ca | wall s |\n|---|---|---|---|---|---|---|---|---|");
    for &mach in &machs {
        for solve in &solves {
            let line = match solve.as_str() {
                "block" => LineSolve3::Block,
                "diag" => LineSolve3::DiagonalCorrected,
                other => panic!("--solve block|diag, got `{other}`"),
            };
            for &cfl in &cfls {
                let mut case = wedge_case3(&engine_case(&example_body(), mach, false, Resolution::Coarse));
                case.numerics.implicit.cfl_max = cfl;
                case.numerics.line_solve = Some(line);
                let t = Instant::now();
                if cpu {
                    let mut s = case.build_solver().expect("wedge solver");
                    let dims = format!("{} x {} x {}", s.grid.ni, s.grid.nj, s.grid.nk);
                    let p = bowshock_core::engine::run3::run3_on(&case, &mut s, false);
                    println!(
                        "| {mach} | {dims} | {solve} (CPU f64) | {cfl} | {} | {:?} | {:.2} | {:.5} | {:.2} |",
                        p.iterations,
                        p.stop_reason,
                        p.drop_orders,
                        p.ca,
                        t.elapsed().as_secs_f64()
                    );
                    continue;
                }
                let mut r = match GpuRunner3::new(&case, prec) {
                    Ok(r) => r,
                    Err(e) => {
                        println!("| {mach} | - | {solve} | {cfl} | error: {e} | | | | |");
                        continue;
                    }
                };
                let g = &r.host.grid;
                let dims = format!("{} x {} x {}", g.ni, g.nj, g.nk);
                match r.run_on(&case, false) {
                    Ok(p) => println!(
                        "| {mach} | {dims} | {solve} | {cfl} | {} | {:?} | {:.2} | {:.5} | {:.2} |",
                        p.iterations,
                        p.stop_reason,
                        p.drop_orders,
                        p.ca,
                        t.elapsed().as_secs_f64()
                    ),
                    Err(e) => println!("| {mach} | {dims} | {solve} | {cfl} | error: {e} | | | | |"),
                }
            }
        }
    }
}
