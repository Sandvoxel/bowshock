//! `mem_items` (DESIGN3D item 48, only builds against the memstats-patched Luma: `tools/memprof/run.sh`): device memory of the 3D GPU step by phase, with Luma's own counters
//! (`Context::debug_mem`: bytes of resident segments + plans, idle pooled bytes) next to the process figure of nvidia-smi, before and after `debug_reclaim` (idle pool given
//! back), for a chain of runners in one process.
//!
//! ```text
//! mem_items --chain sector[,wedge,half,..] [--sub N] [--keep] [--iters N]
//! ```
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

fn row(ctx: &luma::Context, tag: &str) {
    ctx.sync().ok();
    let (res, idle, n) = ctx.debug_mem();
    let before = smi();
    ctx.debug_reclaim().ok();
    ctx.sync().ok();
    std::thread::sleep(std::time::Duration::from_millis(300));
    let after = smi();
    println!("MEMITEM {tag:<64} smi {before:>6} MiB  luma resident {:>6} MiB  idle pool {:>6} MiB ({n} buffers)  smi after reclaim {after:>6} MiB", res >> 20, idle >> 20);
    if let Some(p) = std::env::var_os("LUMA_MEMSTATS") {
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(p) {
            let _ = writeln!(f, "ROW {tag} smi={before} resident={res} idle={idle} smi_after_reclaim={after}");
        }
    }
}

fn case_of(which: &str, mach: f64, sub: Option<usize>) -> Result<Case3, String> {
    let fins = Fins3 { count: 4, root_chord: 0.5, tip_chord: 0.5, span: 0.5, sweep: 0.0, thickness: 0.01, te_to_base: 0.0, profile: FinProfile::DoubleWedge, planform: None };
    let flow = WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let mut case = match which {
        "wedge" => {
            let body = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] };
            Case3::wedge_of(&engine_case(&body, 0.5, true, Resolution::Coarse), 0)
        }
        "anfwedge" => Case3::wedge(mach, Case3::anf_body(), 0).with_tier(Tier::Coarse(flow)),
        "half" => {
            let mut c = Case3::base_half(mach, 2f64.to_radians(), Case3::anf_body(), Some(fins), Tier::Lift(flow), 0);
            c.grid.domain = Domain3::Half { phase: FinPhase::Between };
            c
        }
        _ => Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow), 0),
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
    row(&ctx, "start (context only)");
    let mut kept: Vec<GpuRunner3> = Vec::new();
    for which in chain.split(',') {
        let case = case_of(which, 2.0, sub)?;
        let mut r = GpuRunner3::new(&case, Precision::F32).map_err(|e| e.to_string())?;
        let cells = r.host.grid.ni * r.host.grid.nj * r.host.grid.nk;
        let persistent: usize = r.gpu.memory_report().iter().map(|(_, b)| b).sum();
        println!("MEMITEM == {which}: {cells} cells ({} x {} x {}), sub_iters {}, ledger of persistent arrays {} MiB ({:.0} B/cell)", r.host.grid.ni, r.host.grid.nj, r.host.grid.nk, case.numerics.implicit.sub_iters, persistent >> 20, persistent as f64 / cells as f64);
        row(&ctx, &format!("{which}: after GpuRunner3::new"));
        r.gpu.run_monitored(1).map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after the compile (run_monitored(1))"));
        r.gpu.run_monitored(iters).map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after run_monitored({iters})"));
        r.wall_forces().map_err(|e| e.to_string())?;
        row(&ctx, &format!("{which}: after wall_forces (eager)"));
        if keep {
            kept.push(r);
        } else {
            drop(r);
            row(&ctx, &format!("{which}: runner dropped"));
        }
    }
    row(&ctx, "end of chain");
    drop(kept);
    row(&ctx, "all runners dropped");
    Ok(())
}
