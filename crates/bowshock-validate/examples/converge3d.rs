//! Time to the stop rule of the 3D implicit scheme on the GPU: `GpuRunner3::run_on` (the run3 stop rule: 6 orders of residual drop, or CN stationary
//! over the window) on one Mach number and incidence, with the implicit parameters that decide the cost of an iteration (`sub_iters`) and its
//! convergence (`cfl_max`) overridable. Used to judge an optimisation that changes the numerics by iterations to stop, not only by us/iteration.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example converge3d -- [--case anf|ogive] [--level 0|1] [--mach 2.0] [--alpha 2.0] [--precision df32|f32]
//!     [--sub-iters 2] [--cfl-max 10] [--max-iter N]
//! ```
//!
//! Prints one `CONVERGE {json}` line: iterations, stop reason, drop in orders, CN, x_cp, wall seconds and ms per iteration.

use bowshock_cfd::gpu::Precision;
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
    if let Some(v) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = v.parse().map_err(|e| format!("--cfl-max: {e}"))?;
    }
    if let Some(v) = value("--max-iter") {
        case.numerics.max_iter = v.parse().map_err(|e| format!("--max-iter: {e}"))?;
    }
    let mut runner = GpuRunner3::new(&case, prec).map_err(|e| format!("{e}"))?;
    let t0 = Instant::now();
    let r = runner.run_on(&case, false).map_err(|e| format!("{e}"))?;
    let wall = t0.elapsed().as_secs_f64();
    println!(
        "CONVERGE {{\"case\":\"{case_name}\",\"level\":{level},\"mach\":{mach},\"alpha_deg\":{alpha},\"precision\":\"{pname}\",\"sub_iters\":{},\"cfl_max\":{},\"iterations\":{},\"stop\":\"{:?}\",\"drop_orders\":{:.2},\"cn\":{:.6},\"x_cp_cal\":{:.4},\"wall_s\":{wall:.2},\"ms_per_iter\":{:.3}}}",
        case.numerics.implicit.sub_iters,
        case.numerics.implicit.cfl_max,
        r.iterations,
        r.stop_reason,
        r.drop_orders,
        r.cn,
        r.x_cp,
        1e3 * wall / r.iterations.max(1) as f64
    );
    Ok(())
}
