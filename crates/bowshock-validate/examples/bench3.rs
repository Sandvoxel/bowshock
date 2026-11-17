//! `bench3`: the time of one step of the 3D GPU solver on the grids of the 5-minute target (DESIGN3D E, F3: the replacement of `bench`, `bench_viscous`).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example bench3 -- --case wedge|sector|anf|all [--precision f32,df32] [--solve block,diag]
//!                                                              [--n N] [--mem] [--no-profile] [--no-perf] [--nk N] [--no-wedge-mode]
//! ```
//!
//! Cases (all at Mach 2, `Scheme3::Implicit`, `cfl_max` 10, `first_order_iters` 0: the timed step is the MUSCL one, from the free stream):
//!
//! * `wedge`: the base-wake grid of the example table at its highest node (the wall-resolved grid sized for Mach 3 of the example rocket: 156 x 140 cells
//!   as the 2D table), one azimuthal cell (`nk` 1) over 6 degrees, the body block a solid box. Run with the k factor (the default, `cfl_max` 10) and in wedge
//!   mode (`GpuSolver3::set_wedge_mode`: no k factor, no `L_k` in the time step, pressure flux through the k planes; `cfl_max` 1, the largest the
//!   iteration without the k factor tolerates, see `tests/wedge3d.rs`; `--no-wedge-mode` skips the second). The step time does not depend on `cfl_max`.
//! * `sector`: the same meridian grid with 20 more cells along the last body segment (176 x 140), 18 azimuthal cells over 45 degrees (a 4-fin sector), and a
//!   one-cell fin stand-in (a solid box on the `k = 0` plane, 28 cells along the body, 60 rows high) so the solid-face masks and the wall folds are in the
//!   step. Euler step on today's `Grid3` (no axis runs, no fin mesher: S1 builds the real one); the cost per cell is what it measures.
//! * `anf`: the Army-Navy Basic Finner half domain of the lift solve (`Case3::new(2, 2 deg, anf_body, anf_fins, 0)`: 210 x 70 x 24 = 352 800 cells).
//!
//! For every (case, precision, line solve): 20 warm-up iterations (compile and trace), then `N` timed iterations of the `iterate_all` replay (best of 3),
//! the per-kernel profile of one step as the `eval` loop launches it (launches, GPU time), a 100-iteration sanity run (the residual is finite), and
//! one ledger line
//!
//! ```text
//! PERF {"case":"3d-wedge-euler-block-nok","precision":"df32","cells":..,"iters":N,"us_per_iter":..,"launches_per_iter":..,"gpu_us_per_iter":..,"wall_s":null,"peak_device_mib":..}
//! ```
//!
//! (`--no-perf` suppresses it). `--mem` samples the device memory of this process (`nvidia-smi --query-compute-apps=pid,used_memory`, every 50 ms from a
//! background `nvidia-smi -lms`, filtered to this pid: other processes and the desktop do not count) and reports the peak of the combination in `peak_device_mib`.
//! A combination runs in a context of its own, but what an earlier combination still holds counts: run one combination per process for an isolated number
//! (`tools/gpuq/gpumem` measures any command the same way).
//! The viscous + SST step drops in by implementing [`Step`] for its solver and adding a case that builds it; nothing else here is Euler-specific.

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::freestream3;
use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_cfd::grid3::{Grid3, SolidBox};
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3};
use bowshock_core::engine::body_grid::base_domain_grid;
use bowshock_core::engine::case3::Case3;
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::{base_grid_params, wedge_grid};
use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// What the benchmark needs of a solver: the 3D Euler `GpuSolver3` today, the viscous + SST solver when it exists.
trait Step {
    /// Back to the free stream (the CFL ramp restarted).
    fn reset(&mut self) -> Result<(), String>;
    /// `n` iterations as `iterate_all` replays (no residual history).
    fn replay(&mut self, n: usize) -> Result<(), String>;
    fn sync(&self) -> Result<(), String>;
    /// `n` iterations with the residual recorded.
    fn run(&mut self, n: usize) -> Result<Vec<f64>, String>;
    /// One step as the `eval` loop launches it.
    fn profile_step(&mut self) -> Result<luma::Profile, String>;
}

impl Step for GpuSolver3 {
    fn reset(&mut self) -> Result<(), String> {
        GpuSolver3::reset(self).map_err(|e| e.to_string())
    }
    fn replay(&mut self, n: usize) -> Result<(), String> {
        self.run_replay(n).map_err(|e| e.to_string())
    }
    fn sync(&self) -> Result<(), String> {
        GpuSolver3::sync(self).map_err(|e| e.to_string())
    }
    fn run(&mut self, n: usize) -> Result<Vec<f64>, String> {
        GpuSolver3::run(self, n).map_err(|e| e.to_string())
    }
    fn profile_step(&mut self) -> Result<luma::Profile, String> {
        GpuSolver3::profile_step(self).map_err(|e| e.to_string())
    }
}

/// A grid and what the step runs on it.
struct BenchCase {
    name: &'static str,
    grid: Grid3,
    alpha_deg: f64,
    /// Default number of timed iterations.
    n: usize,
    /// Whether wedge mode applies (`nk == 1`).
    wedge: bool,
}

impl BenchCase {
    fn cells(&self) -> usize {
        self.grid.ni * self.grid.nj * self.grid.nk
    }

    fn config(&self, solve: LineSolve3, wedge_mode: bool) -> Config3 {
        let params = ImplicitParams { cfl_max: if wedge_mode { 1.0 } else { 10.0 }, line_solve: solve, ..ImplicitParams::DEFAULT };
        Config3 {
            freestream: freestream3(2.0, self.alpha_deg.to_radians()),
            cfl: 0.5,
            limiter: Limiter::Minmod,
            first_order_iters: 0,
            low_mach: false,
            scheme: Scheme3::Implicit(params),
        }
    }
}

/// The example rocket's body (`analyze/cfd/plan.rs::plan`): tangent ogive 0.2286 x 0.0762 m and a 0.9134 m tube.
fn example_body() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] }
}

fn wedge_case(nk: usize) -> BenchCase {
    let p = base_grid_params(&example_body(), 3.0, true, Resolution::Coarse);
    BenchCase { name: "wedge", grid: wedge_grid(&p, nk, 6.0), alpha_deg: 0.0, n: 1000, wedge: nk == 1 }
}

fn sector_case(nk: usize) -> BenchCase {
    let mut p = base_grid_params(&example_body(), 3.0, true, Resolution::Coarse);
    *p.n_seg.last_mut().expect("one aft segment") += 20;
    let (_, block) = base_domain_grid(&p).expect("base grid");
    let mut grid = wedge_grid(&p, nk, 45.0);
    // the fin stand-in: a one-cell layer on the k = 0 plane over the last 28 cells of the body, 60 rows high
    grid.solid.push(SolidBox { i: block.i_end - 30..block.i_end - 2, j: block.j_end..block.j_end + 60, k: 0..1 });
    BenchCase { name: "sector", grid, alpha_deg: 0.0, n: 200, wedge: false }
}

fn anf_case() -> Result<BenchCase, String> {
    let case = Case3::new(2.0, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), 0);
    let host = case.build_solver().map_err(|e| format!("ANF grid: {e}"))?;
    Ok(BenchCase { name: "anf", grid: host.grid.clone(), alpha_deg: 2.0, n: 200, wedge: false })
}

// ---------------------------------------------------------------------------------------------------------------------------------
// device memory

/// Peak of the device memory of this process (`nvidia-smi --query-compute-apps=pid,used_memory`, MiB) over the life of the sampler, from a background
/// `nvidia-smi -lms` that prints every process; lines of other pids are ignored. Zero while this process has no device context yet.
struct MemSampler {
    child: Child,
    peak: Arc<AtomicU64>,
    reader: Option<std::thread::JoinHandle<()>>,
}

impl MemSampler {
    fn start() -> Option<MemSampler> {
        let mut child = Command::new("nvidia-smi")
            .args(["--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits", "-lms", "50"])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .ok()?;
        let out = child.stdout.take()?;
        let peak = Arc::new(AtomicU64::new(0));
        let p = peak.clone();
        let me = u64::from(std::process::id());
        let reader = std::thread::spawn(move || {
            for line in BufReader::new(out).lines().map_while(Result::ok) {
                let mut it = line.split(',').map(|x| x.trim().parse::<u64>());
                if let (Some(Ok(pid)), Some(Ok(mib))) = (it.next(), it.next()) {
                    if pid == me {
                        p.fetch_max(mib, Ordering::Relaxed);
                    }
                }
            }
        });
        Some(MemSampler { child, peak, reader: Some(reader) })
    }

    /// Stops sampling (after one more sample) and returns the peak in MiB.
    fn stop(mut self) -> u64 {
        std::thread::sleep(Duration::from_millis(150));
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(r) = self.reader.take() {
            let _ = r.join();
        }
        self.peak.load(Ordering::Relaxed)
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------

fn best_of<F: FnMut() -> Result<f64, String>>(reps: usize, mut f: F) -> Result<f64, String> {
    let mut best = f64::INFINITY;
    for _ in 0..reps {
        best = best.min(f()?);
    }
    Ok(best)
}

/// `Profile` with the kernel labels cut to 90 characters.
fn compact(p: &luma::Profile) -> String {
    let mut out = format!("{:<92} {:>8} {:>10} {:>10}\n", "kernel", "launches", "total us", "mean us");
    for k in &p.kernels {
        let label: String = k.label.chars().take(90).collect();
        out += &format!("{label:<92} {:>8} {:>10.1} {:>10.1}\n", k.launches, k.total.as_secs_f64() * 1e6, k.mean().as_secs_f64() * 1e6);
    }
    out
}

struct Result3 {
    case: String,
    cells: usize,
    precision: &'static str,
    solve: &'static str,
    wedge_mode: bool,
    us: f64,
    launches: u32,
    gpu_us: f64,
    peak_mib: Option<u64>,
    first_call_s: f64,
}

struct Opts {
    perf: bool,
    profile: bool,
    mem: bool,
    n: Option<usize>,
}

/// The measurement of a solver: first call (compile, trace, 20 iterations), us / iteration of the `iterate_all` replay of `n` (best of 3, after 20 warm-up
/// iterations from the free stream), the profile of one step as the `eval` loop launches it, and the last residual of a 100-iteration run from the free
/// stream (a sanity check that the iteration is alive).
fn measure<S: Step>(s: &mut S, n: usize) -> Result<(f64, f64, luma::Profile, f64), String> {
    let t0 = Instant::now();
    s.replay(20)?;
    s.sync()?;
    let first_call_s = t0.elapsed().as_secs_f64();
    let us = best_of(3, || {
        s.reset()?;
        s.replay(20)?;
        s.sync()?;
        let t0 = Instant::now();
        s.replay(n)?;
        s.sync()?;
        Ok(1e6 * t0.elapsed().as_secs_f64() / n as f64)
    })?;
    s.reset()?;
    s.replay(20)?;
    let p = s.profile_step()?;
    s.reset()?;
    let res = s.run(100)?;
    Ok((first_call_s, us, p, res.last().copied().unwrap_or(f64::NAN)))
}

/// One (case, precision, line solve, wedge mode): context of its own, kernels built, timed, profiled, memory sampled.
fn bench_combo(
    case: &BenchCase,
    precision: (Precision, &'static str),
    solve: (LineSolve3, &'static str),
    wedge_mode: bool,
    o: &Opts,
) -> Result<Result3, String> {
    let n = o.n.unwrap_or(case.n);
    let sampler = if o.mem { MemSampler::start() } else { None };
    let ctx = luma::Context::new().map_err(|e| format!("no GPU context: {e}"))?;
    let cfg = case.config(solve.0, wedge_mode);
    let mut s = GpuSolver3::new(&ctx, &case.grid, &cfg, precision.0).map_err(|e| format!("{}: {e}", precision.1))?;
    s.set_wedge_mode(wedge_mode).map_err(|e| e.to_string())?;
    let (first_call_s, us, p, last) = measure(&mut s, n)?;
    if o.profile {
        println!(
            "\n### {} {} {} {}{} - one implicit step as the eval loop launches it: {} kernel launches, {:.0} us GPU time, {} fusion cuts\n```\n{}```",
            case.name,
            precision.1,
            solve.1,
            case.grid.nk,
            if wedge_mode { " (wedge mode)" } else { "" },
            p.launches(),
            p.total().as_secs_f64() * 1e6,
            p.splits.len(),
            compact(&p)
        );
    }
    if !last.is_finite() {
        return Err(format!("{} {} {}: residual {last} after 100 iterations", case.name, precision.1, solve.1));
    }
    let peak_mib = sampler.map(MemSampler::stop);
    Ok(Result3 {
        case: format!("{}{}", case.name, if wedge_mode { "-nok" } else { "" }),
        cells: case.cells(),
        precision: precision.1,
        solve: solve.1,
        wedge_mode,
        us,
        launches: p.launches(),
        gpu_us: p.total().as_secs_f64() * 1e6,
        peak_mib,
        first_call_s,
    })
}

fn parse_list<T: Copy>(arg: Option<String>, all: &[(&'static str, T)], what: &str) -> Result<Vec<(T, &'static str)>, String> {
    let wanted: Vec<String> = match arg {
        Some(s) => s.split(',').map(|x| x.trim().to_string()).collect(),
        None => all.iter().map(|(n, _)| n.to_string()).collect(),
    };
    wanted.iter().map(|w| all.iter().find(|(n, _)| n == w).map(|&(n, v)| (v, n)).ok_or_else(|| format!("{what}: `{w}`"))).collect()
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().any(|a| a == name);
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let which = value("--case")
        .ok_or("usage: bench3 --case wedge|sector|anf|all [--precision f32,df32] [--solve block,diag] [--n N] [--mem] [--no-profile] [--no-perf]")?;
    let precisions = parse_list(value("--precision"), &[("f32", Precision::F32), ("df32", Precision::Extended)], "--precision f32|df32")?;
    let solves = parse_list(value("--solve"), &[("block", LineSolve3::Block), ("diag", LineSolve3::DiagonalCorrected)], "--solve block|diag")?;
    let opts = Opts {
        perf: !flag("--no-perf"),
        profile: !flag("--no-profile"),
        mem: flag("--mem"),
        n: value("--n").map(|n| n.parse::<usize>().map_err(|e| format!("--n: {e}"))).transpose()?,
    };
    let nk = |default: usize| value("--nk").map_or(Ok(default), |v| v.parse::<usize>().map_err(|e| format!("--nk: {e}")));
    let mut cases = Vec::new();
    for c in if which == "all" { vec!["wedge", "sector", "anf"] } else { vec![which.as_str()] } {
        cases.push(match c {
            "wedge" => wedge_case(nk(1)?),
            "sector" => sector_case(nk(18)?),
            "anf" => anf_case()?,
            other => return Err(format!("--case wedge|sector|anf|all, got `{other}`")),
        });
    }
    if opts.mem && MemSampler::start().map(MemSampler::stop).is_none() {
        return Err("--mem needs nvidia-smi".into());
    }
    let mut results = Vec::new();
    for case in &cases {
        eprintln!(
            "{}: grid {} x {} x {} = {} cells, {} solid box(es), {:.1} MiB of f32 state",
            case.name,
            case.grid.ni,
            case.grid.nj,
            case.grid.nk,
            case.cells(),
            case.grid.solid.len(),
            case.cells() as f64 * 5.0 * 4.0 / 1048576.0
        );
        for &precision in &precisions {
            for &solve in &solves {
                for wedge_mode in if case.wedge && !flag("--no-wedge-mode") { vec![false, true] } else { vec![false] } {
                    let r = bench_combo(case, precision, solve, wedge_mode, &opts)?;
                    println!(
                        "{} {} {} {}: {:.1} us / iteration (replay of {} after 20 warm-up, best of 3), {} launches / {:.1} us GPU per step, first call {:.1} s{}",
                        r.case,
                        r.precision,
                        r.solve,
                        r.cells,
                        r.us,
                        opts.n.unwrap_or(case.n),
                        r.launches,
                        r.gpu_us,
                        r.first_call_s,
                        r.peak_mib.map_or(String::new(), |m| format!(", peak device memory of this process {m} MiB"))
                    );
                    if opts.perf {
                        println!(
                            "PERF {{\"case\":\"3d-{}-euler-{}{}\",\"precision\":\"{}\",\"cells\":{},\"iters\":{},\"us_per_iter\":{:.3},\"launches_per_iter\":{},\"gpu_us_per_iter\":{:.3},\"wall_s\":null{}}}",
                            case.name,
                            r.solve,
                            if r.wedge_mode { "-nok" } else { "" },
                            r.precision,
                            r.cells,
                            opts.n.unwrap_or(case.n),
                            r.us,
                            r.launches,
                            r.gpu_us,
                            r.peak_mib.map_or(String::new(), |m| format!(",\"peak_device_mib\":{m}"))
                        );
                    }
                    results.push(r);
                }
            }
        }
    }
    println!("\n| case | cells | precision | line solve | wedge mode | us / iter | launches | GPU us / step | peak MiB (this process) |\n|---|---|---|---|---|---|---|---|---|");
    for r in &results {
        println!(
            "| {} | {} | {} | {} | {} | {:.0} | {} | {:.0} | {} |",
            r.case.trim_end_matches("-nok"),
            r.cells,
            r.precision,
            r.solve,
            if r.wedge_mode { "no k factor" } else { "-" },
            r.us,
            r.launches,
            r.gpu_us,
            r.peak_mib.map_or("-".to_string(), |m| m.to_string())
        );
    }
    Ok(())
}
