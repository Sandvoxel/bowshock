//! Time to the stop rule of the 3D implicit scheme on the GPU: `GpuRunner3::run_on` (the run3 stop rule: 6 orders of residual drop, or CN stationary
//! over the window) on one Mach number and incidence, with the implicit parameters that decide the cost of an iteration (`sub_iters`) and its
//! convergence (`cfl_max`) overridable. Used to judge an optimisation that changes the numerics by iterations to stop, not only by us/iteration.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example converge3d -- [--case anf|ogive] [--level 0|1] [--mach 2.0] [--alpha 2.0] [--precision df32|f32]
//!     [--sub-iters 2] [--cfl-max 10] [--line-solve block|diag] [--max-iter N] [--window W] [--stat-min N] [--repeat R]
//! ```
//!
//! `--line-solve` selects the 3D line solve (default: the product's, `Numerics3::auto`: `diag` = `LineSolve3::DiagonalCorrected`; `block` = the block scheme with
//! `--sub-iters` passes). Prints one `CONVERGE {json}` line per run: iterations, stop reason, drop in orders, CN, x_cp, wall seconds and ms per iteration. `--repeat R`
//! runs the same case R times from the free stream in one process (the first run includes compile and trace; the later ones are warm).

use bowshock_cfd::gpu::Precision;
use bowshock_cfd::implicit::LineSolve3;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::gpu3::GpuRunner3;
use std::time::Instant;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let num = |name: &str, default: f64| value(name).map_or(Ok(default), |v| v.parse::<f64>().map_err(|e| format!("{name}: {e}")));
    let case_name = value("--case").unwrap_or_else(|| "anf".into());
    let level = num("--level", 0.0)? as u32;
    let (mach, alpha) = (num("--mach", 2.0)?, num("--alpha", 2.0)?);
    let (body, fins) = match case_name.as_str() {
        "anf" => (Case3::anf_body(), Some(Case3::anf_fins())),
        "ogive" => {
            let nose = Body { nose: Nose::TangentOgive { fineness: 3.0 } };
            let l = nose.nose_length();
            (BaseBody::cylinder(nose, 10.0 - l), None)
        }
        other => return Err(format!("--case anf|ogive, got `{other}`")),
    };
    let (prec, pname) = match value("--precision").as_deref().unwrap_or("df32") {
        "f32" => (Precision::F32, "f32"),
        "df32" => (Precision::Extended, "df32"),
        other => return Err(format!("--precision f32|df32, got `{other}`")),
    };
    let mut case = Case3::new(mach, alpha.to_radians(), body, fins, level);
    if let Some(v) = value("--sub-iters") {
        case.numerics.implicit.sub_iters = v.parse().map_err(|e| format!("--sub-iters: {e}"))?;
    }
    if let Some(v) = value("--line-solve") {
        case.numerics.implicit.line_solve = match v.as_str() {
            "block" => LineSolve3::Block,
            "diag" => LineSolve3::DiagonalCorrected,
            other => return Err(format!("--line-solve block|diag, got `{other}`")),
        };
    }
    if let Some(v) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = v.parse().map_err(|e| format!("--cfl-max: {e}"))?;
    }
    if let Some(v) = value("--max-iter") {
        case.numerics.max_iter = v.parse().map_err(|e| format!("--max-iter: {e}"))?;
    }
    if let Some(v) = value("--window") {
        case.numerics.window = v.parse().map_err(|e| format!("--window: {e}"))?;
    }
    if let Some(v) = value("--stat-min") {
        case.numerics.stationary_min_iters = v.parse().map_err(|e| format!("--stat-min: {e}"))?;
    }
    let mut runner = GpuRunner3::new(&case, prec).map_err(|e| format!("{e}"))?;
    let repeat = num("--repeat", 1.0)? as usize;
    for rep in 0..repeat {
        let t0 = Instant::now();
        let r = runner.run_on(&case, false).map_err(|e| format!("{e}"))?;
        let wall = t0.elapsed().as_secs_f64();
        println!(
        "CONVERGE {{\"rep\":{rep},\"case\":\"{case_name}\",\"level\":{level},\"mach\":{mach},\"alpha_deg\":{alpha},\"precision\":\"{pname}\",\"line_solve\":\"{:?}\",\"sub_iters\":{},\"cfl_max\":{},\"iterations\":{},\"stop\":\"{:?}\",\"drop_orders\":{:.2},\"cn\":{:.6},\"x_cp_cal\":{:.4},\"wall_s\":{wall:.3},\"ms_per_iter\":{:.3}}}",
        case.numerics.implicit.line_solve,
        case.numerics.implicit.sub_iters,
        case.numerics.implicit.cfl_max,
        r.iterations,
        r.stop_reason,
        r.drop_orders,
        r.cn,
        r.x_cp,
        1e3 * wall / r.iterations.max(1) as f64
    );
    }
    Ok(())
}
