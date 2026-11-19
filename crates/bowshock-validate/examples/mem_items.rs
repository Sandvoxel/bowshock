//! `mem_items` (DESIGN3D items 48 and 49, `tools/memprof/run.sh`): device memory of the 3D GPU step by phase, with Luma's own counters
//! (`Context::memory_stats`: resident bytes of arrays + plans, idle pooled bytes) next to the process figure of nvidia-smi (sampled every 40 ms: the peak since the last row is the
//! compile peak), before and after `Context::trim` (idle pool given back), for a chain of runners in one process.
//!
//! ```text
//! mem_items --chain anf[@SUB][,gf1,anfwedge,half,anfhalf,wedge,..] [--sub N] [--keep] [--iters N] [--reclaim] [--ext] [--run-on N] [--snaps N]
//! ```
//! `anf` (default) is `finned_table --anf --tier coarse`; `gf1` the G-F1 sector of `mem_stages`; `anfhalf` the ANF half domain at 2 degrees (Lift tier, `Between`); `half` the same with the G-F1 fins.
//! `--reclaim` gives the idle pool back after every row (measures the steady state; without it the rows show the natural footprint, pool included).
//! `--ext` builds the runners in extended (double-float) precision.
//! `--keep` holds every runner of the chain alive (as the product keeps a table's solver); without it each runner is dropped before the next is built.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::{AftSpec, BodySpec, FinProfile, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::engine_case;
use std::process::Command;

fn smi() -> u64 {
    let out = Command::new("nvidia-smi").args(["--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"]).output().ok();
    let me = u64::from(std::process::id());
    out.map(|o| {
        String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| {
                let mut it = l.split(',').map(|x| x.trim().parse::<u64>());
                match (it.next(), it.next()) {
                    (Some(Ok(pid)), Some(Ok(m))) if pid == me => Some(m),
                    _ => None,
                }
            })
            .max()
            .unwrap_or(0)
    })
    .unwrap_or(0)
}

static PEAK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Samples the process figure of nvidia-smi in the background and keeps the maximum since the last row.
fn start_sampler() {
    std::thread::spawn(|| loop {
        PEAK.fetch_max(smi(), std::sync::atomic::Ordering::Relaxed);
        std::thread::sleep(std::time::Duration::from_millis(40));
    });
}

fn row(ctx: &luma::Context, tag: &str, reclaim: bool) {
    ctx.sync().ok();
    let m = ctx.memory_stats();
    let (res, idle) = (m.resident_bytes, m.pooled_bytes);
    let before = smi();
    let peak = PEAK.swap(0, std::sync::atomic::Ordering::Relaxed).max(before);
    let after = if reclaim {
        ctx.trim().ok();
        ctx.sync().ok();
        std::thread::sleep(std::time::Duration::from_millis(300));
        smi()
    } else {
        0
    };
    println!(
        "MEMITEM {tag:<64} smi {before:>6} MiB (peak since last row {peak:>6})  luma resident {:>6} MiB  idle pool {:>6} MiB  smi after reclaim {after:>6} MiB",
        res >> 20,
        idle >> 20
    );
}

fn case_of(which: &str, mach: f64, sub: Option<usize>) -> Result<Case3, String> {
    let fins = Fins3 {
        count: 4,
        root_chord: 0.5,
        tip_chord: 0.5,
        span: 0.5,
        sweep: 0.0,
        thickness: 0.01,
        te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
        planform: None,
    };
    let flow = WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    // `finned_table --anf --tier coarse`: the Army-Navy Basic Finner with the sourced fin section, grid sized for Mach 3.147
    let anf_flow = WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: 3.147, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let mut case = match which {
        "wedge" => {
            let body =
                BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] };
            Case3::wedge_of(&engine_case(&body, 0.5, true, Resolution::Coarse), 0)
        }
        "anfwedge" => Case3::wedge(mach, Case3::anf_body(), 0).with_tier(Tier::Coarse(flow)),
        "half" => {
            let mut c = Case3::base_half(mach, 2f64.to_radians(), Case3::anf_body(), Some(fins), Tier::Lift(flow), 0);
            c.grid.domain = Domain3::Half { phase: FinPhase::Between };
            c
        }
        "gf1" => Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow), 0),
        "anfhalf" => {
            let mut c = Case3::base_half(mach, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), Tier::Lift(anf_flow), 0);
            c.grid.domain = Domain3::Half { phase: FinPhase::Between };
            c
        }
        _ => Case3::sector(mach, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(anf_flow), 0),
    };
    if let Some(s) = sub {
        case.numerics.implicit.sub_iters = s;
    }
    Ok(case)
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let chain = value("--chain").unwrap_or_else(|| "sector".into());
    let keep = args.iter().any(|a| a == "--keep");
    let sub = value("--sub").map(|v| v.parse::<usize>().map_err(|e| e.to_string())).transpose()?;
    let iters: usize = value("--iters").map_or(Ok(50), |v| v.parse().map_err(|e| format!("{e}")))?;
    let ctx = luma::default_context().map_err(|e| e.to_string())?;
    let reclaim = args.iter().any(|a| a == "--reclaim");
    start_sampler();
    row(&ctx, "start (context only)", reclaim);
    let mut kept: Vec<GpuRunner3> = Vec::new();
    for item in chain.split(',') {
        // `name@N`: `sub_iters` N for this item
        let (which, sub_item) = match item.split_once('@') {
            Some((w, n)) => (w, Some(n.parse::<usize>().map_err(|e| e.to_string())?)),
            None => (item, sub),
        };
        let case = case_of(which, 2.0, sub_item)?;
        let mut r = GpuRunner3::new(&case, if args.iter().any(|a| a == "--ext") { Precision::Extended } else { Precision::F32 }).map_err(|e| e.to_string())?;
        let cells = r.host.grid.ni * r.host.grid.nj * r.host.grid.nk;
        let persistent: usize = r.gpu.memory_report().iter().map(|(_, b)| b).sum();
        println!(
            "MEMITEM == {which}: {cells} cells ({} x {} x {}), sub_iters {}, ledger of persistent arrays {} MiB ({:.0} B/cell)",
            r.host.grid.ni,
            r.host.grid.nj,
            r.host.grid.nk,
            case.numerics.implicit.sub_iters,
            persistent >> 20,
            persistent as f64 / cells as f64
        );
        row(&ctx, &format!("{which}: after GpuRunner3::new"), reclaim);
        r.gpu.run_monitored(1).map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after the compile (run_monitored(1))"), reclaim);
        r.gpu.run_monitored(iters).map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after run_monitored({iters})"), reclaim);
        r.wall_forces().map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after wall_forces (eager)"), reclaim);
        // the CFL controller's snapshots (`CflControl::accept`): one `GpuSolver3::snapshot()` after each chunk of 50 iterations, all kept
        if let Some(n) = value("--snaps").map(|v| v.parse::<usize>().map_err(|e| e.to_string())).transpose()? {
            let mut snaps = Vec::new();
            for k in 0..n {
                r.gpu.run_monitored(50).map_err(|e| e.to_string())?;
                snaps.push(r.gpu.snapshot().map_err(|e| e.to_string())?);
                row(&ctx, &format!("{which}: {} snapshots kept after {} chunks", k + 1, k + 1), reclaim);
            }
            drop(snaps);
            row(&ctx, &format!("{which}: snapshots dropped"), reclaim);
        }
        // the product path: `run_on` (CFL controller snapshots, limiter phases, the stop rule's force reads) for N iterations from the free stream
        if let Some(n) = value("--run-on").map(|v| v.parse::<usize>().map_err(|e| e.to_string())).transpose()? {
            let mut c = case.clone();
            c.numerics.max_iter = n;
            let p = r.run_on(&c, false).map_err(|e| e.to_string())?;
            row(
                &ctx,
                &format!("{which}: after run_on({n}): {} iterations, stop {:?}, {} cfl events", p.iterations, p.stop_reason, p.cfl_events.len()),
                reclaim,
            );
        }
        if keep {
            kept.push(r);
        } else {
            drop(r);
            row(&ctx, &format!("{which}: runner dropped"), reclaim);
        }
    }
    row(&ctx, "end of chain", reclaim);
    drop(kept);
    row(&ctx, "all runners dropped", reclaim);
    Ok(())
}
