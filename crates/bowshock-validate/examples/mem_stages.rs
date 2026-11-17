//! `mem_stages`: the device memory of this process after each stage of a GPU runner (nvidia-smi per-process figure, one sample after a `sync`), to see where the transient
//! buffers of the 3D step come from: the context, the persistent arrays (`GpuRunner3::new`), the compile of the loop (first `run_monitored`), later chunks, the eager
//! `lift_forces` evaluation, the eval-per-iteration loop.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example mem_stages -- --case sector|wedge [--mach 2.0] [--sub N]
//! ```
//!
//! `sector` is the G-F1 case (ANF body, four thin fins, Coarse tier) as `bench3 --case sector-rans`; `wedge` the example-table wedge at Mach 0.5.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::{AftSpec, BodySpec, FinProfile, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::engine_case;
use std::process::Command;

/// The device memory of this process in MiB (0 while it has no context).
fn mib() -> u64 {
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

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let which = value("--case").unwrap_or_else(|| "sector".into());
    let mach: f64 = value("--mach").map_or(Ok(2.0), |v| v.parse().map_err(|e| format!("--mach: {e}")))?;
    let case = if which == "wedge" {
        let body =
            BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] };
        Case3::wedge_of(&engine_case(&body, 0.5, true, Resolution::Coarse), 0)
    } else {
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
        Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow), 0)
    };
    let mut case = case;
    if let Some(s) = value("--sub") {
        case.numerics.implicit.sub_iters = s.parse().map_err(|e| format!("--sub: {e}"))?;
    }
    let row = |stage: &str, base: u64| println!("MEMSTAGE {which}: {stage:<58} {:>6} MiB", mib().saturating_sub(base));
    println!("MEMSTAGE {which}: start {} MiB (process)", mib());
    let mut r = GpuRunner3::new(&case, Precision::F32).map_err(|e| e.to_string())?;
    let cells = r.host.grid.ni * r.host.grid.nj * r.host.grid.nk;
    r.gpu.context().sync().map_err(|e| e.to_string())?;
    let after_new = mib();
    println!("MEMSTAGE {which}: {cells} cells; after GpuRunner3::new (context + persistent arrays + initial state): {after_new} MiB");
    let persistent: usize = r.gpu.memory_report().iter().map(|(_, b)| b).sum();
    println!("MEMSTAGE {which}: ledger of the persistent arrays {} MiB", persistent / 1048576);
    // the loop as the runner runs it
    r.gpu.run_monitored(1).map_err(|e| e.to_string())?;
    r.gpu.context().sync().map_err(|e| e.to_string())?;
    row("first run_monitored(1): compile + record the loop (+)", after_new);
    r.gpu.run_monitored(50).map_err(|e| e.to_string())?;
    r.gpu.context().sync().map_err(|e| e.to_string())?;
    row("run_monitored(50) after it (+)", after_new);
    // the limiter phase switch of a run (first-order start-up, then the MUSCL loop: a second compiled loop replaces the first)
    let mut cfg = r.host.cfg.clone();
    cfg.first_order_iters = 10;
    r.gpu.set_config(&cfg).map_err(|e| e.to_string())?;
    r.gpu.set_iter(0);
    r.gpu.run_monitored(60).map_err(|e| e.to_string())?;
    r.gpu.context().sync().map_err(|e| e.to_string())?;
    row("first-order (10 its) then MUSCL (50): two compiled loops in turn (+)", after_new);
    r.gpu.run_replay(20).map_err(|e| e.to_string())?;
    r.gpu.context().sync().map_err(|e| e.to_string())?;
    row("run_replay(20): a second recorded loop (+)", after_new);
    r.wall_forces().map_err(|e| e.to_string())?;
    row("eager force evaluation (wall_forces, viscous_wall_forces) (+)", after_new);
    Ok(())
}
