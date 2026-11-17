//! `plates_gpu`: the plate gates of `bowshock_cfd::plate3` (G-V1 laminar plate, G-V2 turbulent plate, the `y+` sweep, G2r rough plate) on the GPU: the viscous SST
//! step of stage S5b (`Case`s of `plate3`, thin 3D box grids, `LineSolve3::Block`), through gpuq.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example plates_gpu -- --case g1|g2|g2r|yplus:5 [--precision f32|df32] [--max-iter N]
//! ```
//!
//! The iteration is the stop rule of the CPU tests: the density residual `drop` orders below its peak (after iteration 20; 4 orders, 5 for G1) or `cf` at the gate
//! station stationary to `1e-5` per 250 iterations (G1 and the sweep points). One line per station: `cf`, the reference (Blasius, `0.37 / log10(5e6 x)^2.584`,
//! Mills & Hang) and the difference; the first-cell `y+`; iterations and wall time. `cf` is the traction of the no-slip wall faces from the viscous face fluxes
//! of the device state (`GpuSolver3::viscous_flux`, `viscous3::wall_tractions`).

use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_cfd::implicit_params::{ImplicitParams, LineSolve3};
use bowshock_cfd::plate3::{blasius, mills_hang, plate_solver3, turbulent_correlation, PlateCase};
use bowshock_cfd::viscous3::wall_tractions;
use std::time::Instant;

/// `cf` and the first-cell `y+` of the plate face nearest to `x_target` (`plate3::station` from the device state).
fn station(gpu: &GpuSolver3, host: &bowshock_cfd::solver3::Solver3, case: &PlateCase, solid: &[bool], x_target: f64) -> Result<(f64, f64, f64), String> {
    let g = &host.grid;
    let r = host.rans().ok_or("viscous host")?;
    let [fvi, fvj, fvk] = gpu.viscous_flux().map_err(|e| e.to_string())?.ok_or("no viscous flux")?;
    let wall = wall_tractions(g, solid, &r.dirs, &fvi, &fvj, &fvk);
    let t = wall
        .iter()
        .filter(|t| t.dir == 1 && t.idx[0] >= case.i_le() && t.n[1] < -0.5)
        .min_by(|a, b| {
            let xa = (g.jface[(a.idx[2] * (g.nj + 1) + a.idx[1]) * g.ni + a.idx[0]].c[0] - x_target).abs();
            let xb = (g.jface[(b.idx[2] * (g.nj + 1) + b.idx[1]) * g.ni + b.idx[0]].c[0] - x_target).abs();
            xa.total_cmp(&xb)
        })
        .ok_or("no plate face")?;
    let [i, j, k] = t.idx;
    let face = &g.jface[(k * (g.nj + 1) + j) * g.ni + i];
    let c = g.cell(i, j, k);
    let q = 0.5 * host.cfg.freestream[0] * case.mach * case.mach;
    let u = gpu.state_f64().map_err(|e| e.to_string())?[c];
    let (rho, p) = (u[0], (bowshock_cfd::gas3::GAMMA - 1.0) * (u[4] - 0.5 * (u[1] * u[1] + u[2] * u[2] + u[3] * u[3]) / u[0]));
    let nu = case.viscous().mu(bowshock_cfd::gas3::GAMMA * p / rho) / rho;
    let d = if r.dwall.is_empty() { (g.xc[c][1] - face.c[1]).abs() } else { r.dwall[c] };
    Ok((face.c[0], t.tau[0] / q, d * (t.tau[0].abs() / rho).sqrt() / nu))
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let which = value("--case").ok_or("usage: plates_gpu --case g1|g2|g2r|yplus:<y+> [--precision f32|df32] [--max-iter N]")?;
    let prec = if value("--precision").as_deref() == Some("f32") { Precision::F32 } else { Precision::Extended };
    let (case, drop, x_gate, stationary): (PlateCase, f64, f64, bool) = match which.as_str() {
        "g1" => (PlateCase::g1(), 5.0, 0.9, true),
        "g2" => (PlateCase::g2(), 4.0, 0.97, false),
        "g2r" => (PlateCase::g2r(), 4.0, 0.97, false),
        s if s.starts_with("yplus:") => (PlateCase::g2_yplus(s[6..].parse().map_err(|e| format!("y+: {e}"))?), 4.0, 0.97, false),
        other => return Err(format!("--case g1|g2|g2r|yplus:<y+>, got `{other}`")),
    };
    // the plate as the `j = 0` boundary wall of the grid (step-free leading edge, as in 2D); `--slab`: the solid slab under it with the step at the leading edge
    let case = if args.iter().any(|a| a == "--slab") { case } else { case.boundary_wall() };
    let max_iter: usize = value("--max-iter").map_or(Ok(if stationary { 6000 } else { 20_000 }), |v| v.parse().map_err(|e| format!("--max-iter: {e}")))?;
    let params = ImplicitParams { line_solve: LineSolve3::Block, ..ImplicitParams::VISCOUS };
    let host = plate_solver3(&case, params);
    if args.iter().any(|a| a == "--cpu") {
        return run_cpu(&which, &case, drop, max_iter, host);
    }
    let solid: Vec<bool> = host.grid.part.iter().map(|&p| p != bowshock_cfd::grid3::PART_FLUID).collect();
    let ctx = luma::Context::new().map_err(|e| e.to_string())?;
    let t0 = Instant::now();
    let mut gpu = GpuSolver3::from_host(&ctx, &host, prec).map_err(|e| e.to_string())?;
    let setup = t0.elapsed().as_secs_f64();
    println!("{which}: {} x {} x {} cells, {:?}, set-up {setup:.1} s", host.grid.ni, host.grid.nj, host.grid.nk, prec);
    let (mut peak, mut last, mut iters, mut converged) = (0.0f64, f64::INFINITY, 0usize, false);
    let mut cf_prev = f64::NAN;
    let t1 = Instant::now();
    'run: while iters < max_iter {
        let h = gpu.run(50.min(max_iter - iters)).map_err(|e| e.to_string())?;
        for r in h {
            iters += 1;
            last = r;
            if !r.is_finite() {
                println!("non-finite residual at iteration {iters}");
                break 'run;
            }
            if iters > 21 {
                peak = peak.max(r);
                if peak / r >= 10f64.powf(drop) {
                    converged = true;
                    break 'run;
                }
            }
        }
        if stationary && iters % 250 == 0 {
            let cf = station(&gpu, &host, &case, &solid, x_gate)?.1;
            if (cf / cf_prev - 1.0).abs() < 1e-5 {
                converged = true;
                break;
            }
            cf_prev = cf;
        }
    }
    let wall = t1.elapsed().as_secs_f64();
    println!(
        "{iters} iterations, {wall:.1} s ({:.2} ms per iteration), density residual {last:.3e} (peak {peak:.3e}, drop {:.2} orders), converged {converged}",
        1e3 * wall / iters as f64,
        (peak / last).log10()
    );
    table(&which, &case, |xt| station(&gpu, &host, &case, &solid, xt))?;
    if let Some((k, mu_t)) = gpu.turb_state().map_err(|e| e.to_string())? {
        let kmax = k.iter().map(|x| x[0]).fold(0.0f64, f64::max);
        let mmax = mu_t.iter().copied().fold(0.0f64, f64::max);
        println!("max k {kmax:.3e}, max mu_t {mmax:.3e}");
    }
    println!(
        "PERF {{\"case\":\"3d-plate-{}-rans-gpu\",\"precision\":\"{}\",\"cells\":{},\"iters\":{iters},\"us_per_iter\":{:.3},\"wall_s\":{wall:.3}}}",
        which.replace(':', ""),
        if prec == Precision::F32 { "f32" } else { "df32" },
        host.grid.ni * host.grid.nj * host.grid.nk,
        1e6 * wall / iters as f64
    );
    Ok(())
}

/// The station table: `cf` and first-cell `y+` of the stations of the case against the reference (Blasius, the turbulent correlation, Mills & Hang).
fn table(which: &str, case: &PlateCase, mut at: impl FnMut(f64) -> Result<(f64, f64, f64), String>) -> Result<(), String> {
    let stations: &[f64] = if which == "g1" {
        &[0.5, 0.9]
    } else if which == "g2" {
        &[0.25, 0.5, 0.76, 0.97]
    } else {
        &[0.97]
    };
    println!("| station x | cf | reference | difference | first-cell y+ |\n|---|---|---|---|---|");
    for &xt in stations {
        let (x, cf, yp) = at(xt)?;
        let reference = if case.ks > 0.0 {
            mills_hang(x, case.ks)
        } else if which == "g1" {
            blasius(case, x)
        } else {
            turbulent_correlation(x)
        };
        println!("| {x:.4} | {cf:.5e} | {reference:.5e} | {:+.2} % | {yp:.2} |", 100.0 * (cf / reference - 1.0));
    }
    Ok(())
}

/// The same run on the CPU (`plate3::run`, the stop rule of the CPU gates), for the table next to the GPU one.
fn run_cpu(which: &str, case: &PlateCase, drop: f64, max_iter: usize, mut host: bowshock_cfd::solver3::Solver3) -> Result<(), String> {
    let t = Instant::now();
    let info = bowshock_cfd::plate3::run(&mut host, drop, max_iter, 0, |_, _| false);
    let wall = t.elapsed().as_secs_f64();
    println!("{which} (CPU): {} iterations, {wall:.1} s, drop {:.2} orders, converged {}", info.iters, info.drop_orders(), info.converged);
    table(which, case, |xt| {
        let s = bowshock_cfd::plate3::station(&host, case, xt);
        Ok((s.x, s.cf, s.y_plus))
    })
}
