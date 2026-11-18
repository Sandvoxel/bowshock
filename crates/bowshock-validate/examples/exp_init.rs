//! `exp_init`: wedge-initialised finned sector (experiment, opt-in). The axisymmetric single-cell wedge of the example body at the same Mach number and tier is solved first
//! (`Case3::wedge(..).with_tier(..)`), its `rho, u, E, k, omega` are transferred to the sector grid ([`bowshock_core::engine::axisym_init`]) and the sector warm-starts from them
//! (`GpuSolver3::set_state_turb` + `GpuRunner3::run_on(case, true)`). `--cold` runs only the plain sector as the baseline.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example exp_init -- --tier coarse|draft|draft-safe --mach 0.9 [--cold] [--max-iter N]
//! ```
//!
//! One `INIT {json}` line at the end; `PROG` lines every 500 iterations of the sector (its stop-rule monitor: CD).

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::axisym_init::axisym_to_sector;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::FinProfile;
use bowshock_core::ProgressSink;
use std::ops::ControlFlow;
use std::time::Instant;

fn example_body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 0.2286 / 0.0762 } }, 0.9134 / 0.0762)
}

/// The finned example rocket's fins: THREE square fins (the example document and the frozen 2D fixture; `tests/grid_census.rs` has four, the first version of this helper copied it).
fn example_fins() -> Fins3 {
    let d = 0.0762;
    Fins3 {
        count: 3,
        root_chord: 0.12 / d,
        tip_chord: 0.05 / d,
        span: 0.0762 / d,
        sweep: 0.06 / d,
        thickness: 0.00318 / d,
        te_to_base: 0.0,
        profile: FinProfile::Square,
        planform: None,
    }
}

fn with_progress(mut case: Case3, label: &'static str, every: usize) -> Case3 {
    case.progress = Some(ProgressSink::new(move |p| {
        if p.iteration % every == 0 {
            println!("PROG {label} it {} cd {:.6} drop {:.2}", p.iteration, p.monitor, p.residual_drop_orders);
        }
        ControlFlow::Continue(())
    }));
    case
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let mach: f64 = value("--mach").ok_or("--mach <M>")?.parse().map_err(|e| format!("--mach: {e}"))?;
    let tier_name = value("--tier").unwrap_or_else(|| "draft".into());
    let cold = args.iter().any(|a| a == "--cold");
    let flow = WallFlow { re_c: 1.7e6, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
    let tier = match tier_name.as_str() {
        "coarse" => Tier::Coarse(flow),
        "draft" | "draft-safe" => Tier::draft(flow),
        other => return Err(format!("--tier coarse|draft|draft-safe, not {other}")),
    };
    let safe = tier_name == "draft-safe";
    let mut sector = Case3::sector(mach, example_body(), example_fins(), tier, 0);
    // the wedge of the same body, Mach number, tier and level: `with_tier` sizes the same wall-resolved meridian, `on_wedge` the single azimuthal cell and the block lines
    let mut wedge = Case3::wedge(mach, example_body(), 0).with_tier(tier).on_wedge();
    if safe {
        sector = sector.draft_subsonic();
        wedge = wedge.draft_subsonic();
    }
    if let Some(n) = value("--max-iter") {
        let n = n.parse().map_err(|e| format!("--max-iter: {e}"))?;
        sector.numerics.max_iter = n;
        wedge.numerics.max_iter = n;
    }
    if let Some(n) = value("--first-order") {
        sector.numerics.first_order_iters = n.parse().map_err(|e| format!("--first-order: {e}"))?;
    }
    // `--fixed N`: the sector runs exactly N iterations (every stop rule off: no range rule, no mean rule) and reports its CD monitor every 50
    let fixed: Option<usize> = value("--fixed").map(|v| v.parse()).transpose().map_err(|e| format!("--fixed: {e}"))?;
    if let Some(n) = fixed {
        sector.numerics.max_iter = n;
        sector.numerics.rel_tol = -1.0;
        sector.numerics.mean_window = 0;
    }
    let sector = with_progress(sector, "sector", if fixed.is_some() { 50 } else { 500 });
    let wedge = with_progress(wedge, "wedge", 500);

    let t_total = Instant::now();
    let (mut wedge_iters, mut wedge_s) = (0usize, 0.0f64);
    let mut sector_t0 = Instant::now();
    let mut init = None;
    if !cold {
        let t = Instant::now();
        let mut wr = GpuRunner3::new(&wedge, Precision::F32).map_err(|e| e.to_string())?;
        let wp = wr.run_on(&wedge, false).map_err(|e| e.to_string())?;
        let u = wr.gpu.state_f64().map_err(|e| e.to_string())?;
        let (turb, _mu_t) = wr.gpu.turb_state().map_err(|e| e.to_string())?.ok_or("the wedge has no SST state")?;
        wedge_s = t.elapsed().as_secs_f64();
        wedge_iters = wp.iterations;
        let wg = &wr.host.grid;
        println!("WEDGE grid {}x{}x{}, {} iterations ({:?}), {wedge_s:.1} s, CD monitor {:.5}", wg.ni, wg.nj, wg.nk, wp.iterations, wp.stop_reason, wp.ca);
        init = Some((u, turb, wr.host.grid.clone()));
        drop(wr);
        sector_t0 = Instant::now();
    }
    let mut runner = GpuRunner3::new(&sector, Precision::F32).map_err(|e| e.to_string())?;
    let sg = &runner.host.grid;
    println!("SECTOR grid {}x{}x{}", sg.ni, sg.nj, sg.nk);
    if let Some((u, turb, wg)) = &init {
        meridian_report(wg, sg);
        let t = Instant::now();
        let fs = bowshock_cfd::gas3::prim_to_cons3(&sector.freestream());
        let turb_fs = [bowshock_cfd::sst::K_INF, bowshock_cfd::sst::omega_inf(flow.re_c)];
        let m = axisym_to_sector(wg, u, turb, sg, fs, turb_fs);
        println!("TRANSFER {:.2} s: max |dx| {:.3e} |dr| {:.3e} (centres), {} fallbacks", t.elapsed().as_secs_f64(), m.max_dx, m.max_dr, m.fallbacks);
        runner.gpu.set_state_turb(&m.u, Some(&m.turb)).map_err(|e| e.to_string())?;
    }
    let p = runner.run_on(&sector, !cold).map_err(|e| e.to_string())?;
    println!("CN {:.3e} (zero incidence: the monitor of the PROG lines is CA = CD)", p.cn);
    let d = runner.drag_parts(&sector).map_err(|e| e.to_string())?;
    let sector_s = sector_t0.elapsed().as_secs_f64();
    let total_s = t_total.elapsed().as_secs_f64();
    println!(
        "INIT {{\"mode\":\"{}\",\"tier\":\"{tier_name}\",\"mach\":{mach},\"wedge_iters\":{wedge_iters},\"wedge_s\":{wedge_s:.1},\"sector_iters\":{},\"sector_s\":{sector_s:.1},\"total_s\":{total_s:.1},\"stop\":\"{:?}\",\"cd\":{:.5},\"body_fr\":{:.5},\"fin_fr\":{:.5},\"fore_p\":{:.5},\"base\":{:.5},\"fin_p\":{:.5}}}",
        if cold { "cold" } else { "wedge" },
        p.iterations,
        p.stop_reason,
        d.total,
        d.body_friction(),
        d.fin_friction(),
        d.nose,
        d.base,
        d.fin_sides + d.fin_edges + d.fin_tips
    );
    Ok(())
}

/// Largest `|dx|` and `|dr|` between the node lines of the wedge's meridian (`k = 0`) and the sector's (its last plane, the mid-gap plane away from the fin: the fin sits on `k = 0`).
fn meridian_report(w: &bowshock_cfd::grid3::Grid3, s: &bowshock_cfd::grid3::Grid3) {
    if (w.ni, w.nj) != (s.ni, s.nj) {
        println!("MERIDIAN dimensions differ: wedge {}x{}, sector {}x{}", w.ni, w.nj, s.ni, s.nj);
        return;
    }
    let (mut dx, mut dr, mut rel) = (0.0f64, 0.0f64, 0.0f64);
    for j in 0..=w.nj {
        for i in 0..=w.ni {
            let a = j * (w.ni + 1) + i;
            let b = (s.nk * (s.nj + 1) + j) * (s.ni + 1) + i;
            dx = dx.max((w.x[a] - s.x[b]).abs());
            let (rw, rs) = (w.y[a].hypot(w.z[a]), s.y[b].hypot(s.z[b]));
            dr = dr.max((rw - rs).abs());
            rel = rel.max((rw - rs).abs() / rw.max(1e-3));
        }
    }
    println!("MERIDIAN node lines wedge k=0 vs sector k=nk: max |dx| {dx:.3e}, max |dr| {dr:.3e} (max relative {rel:.3e})");
}
