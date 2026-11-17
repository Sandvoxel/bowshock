//! `bench3`: the time of one step of the 3D GPU solver on the grids of the 5-minute target (DESIGN3D E, F3: the replacement of `bench`, `bench_viscous`).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example bench3 -- --case wedge|sector|anf|all [--precision f32,df32] [--solve block,diag]
//!                                                              [--n N] [--mem] [--no-profile] [--no-perf] [--nk N]
//! ```
//!
//! Cases (all at Mach 2, `Scheme3::Implicit`, `cfl_max` 10, `first_order_iters` 0: the timed step is the MUSCL one, from the free stream). The grids are
//! the S1 ones (`body_grid3`: revolved base-wake grid with dead cells and axis runs, the fin sectors, the wedge with its pressure flux and source block):
//!
//! * `wedge`: `Case3::wedge` on the base-wake grid of the example table's highest node (the wall-resolved grid sized for Mach 3 of the example rocket: 156 x 140
//!   cells as the 2D table), one azimuthal cell over `WEDGE_ANGLE` (no k factor, no `L_k` in the time step, pressure flux through the k planes, the source
//!   block on the j diagonal). The step time does not depend on `cfl_max`.
//! * `sector`: `Case3::sector` of the Army-Navy Basic Finner (4 fins, `[0, pi/4]`), `nk` (18) azimuthal cells, on the sting grid with the real fin boxes.
//! * `anf`: the Army-Navy Basic Finner half domain of the lift solve (`Case3::new(2, 2 deg, anf_body, anf_fins, 0)`: 210 x 70 x 24 = 352 800 cells).
//!
//! For every (case, precision, line solve): 20 warm-up iterations (compile and trace), then `N` timed iterations of the `iterate_all` replay (best of 3),
//! the per-kernel profile of one step as the `eval` loop launches it (launches, GPU time), a 100-iteration sanity run (the residual is finite), and
//! one ledger line
//!
//! ```text
//! PERF {"case":"3d-wedge-euler-block","precision":"df32","cells":..,"iters":N,"us_per_iter":..,"launches_per_iter":..,"gpu_us_per_iter":..,"wall_s":null,"peak_device_mib":..}
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
use bowshock_cfd::grid3::Grid3;
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3};
use bowshock_core::engine::case3::{Case3, Tier};
use bowshock_core::engine::grid3::body_grid3;
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::{engine_case, wedge_case3};
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
}

impl BenchCase {
    fn cells(&self) -> usize {
        self.grid.ni * self.grid.nj * self.grid.nk
    }

    fn config(&self, solve: LineSolve3) -> Config3 {
        let params = ImplicitParams { cfl_max: 10.0, line_solve: solve, ..ImplicitParams::DEFAULT };
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

/// The wedge of the example table's highest node: the wall-resolved base-wake grid sized for Mach 3 (156 x 140 cells as the 2D table), one cell over
/// `WEDGE_ANGLE`.
fn wedge_case() -> Result<BenchCase, String> {
    let c2 = engine_case(&example_body(), 3.0, true, Resolution::Coarse);
    let case = wedge_case3(&c2);
    let grid = body_grid3(&case.grid).map_err(|e| format!("wedge grid: {e}"))?;
    Ok(BenchCase { name: "wedge", grid, alpha_deg: 0.0, n: 1000 })
}

/// The real fin sector: the Army-Navy Basic Finner (4 fins, even `N`) as the sector `[0, pi/4]` on the sting grid, `nk` azimuthal cells.
fn sector_case(nk: usize) -> Result<BenchCase, String> {
    let mut case = Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Euler, 0);
    case.grid.nk = nk;
    let grid = body_grid3(&case.grid).map_err(|e| format!("sector grid: {e}"))?;
    Ok(BenchCase { name: "sector", grid, alpha_deg: 0.0, n: 200 })
}

fn anf_case() -> Result<BenchCase, String> {
    let case = Case3::new(2.0, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), 0);
    let host = case.build_solver().map_err(|e| format!("ANF grid: {e}"))?;
    Ok(BenchCase { name: "anf", grid: host.grid.clone(), alpha_deg: 2.0, n: 200 })
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
    /// Time a combination whose 100-iteration sanity run diverges (the step cost does not depend on the values), with a warning.
    allow_nonfinite: bool,
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
fn bench_combo(case: &BenchCase, precision: (Precision, &'static str), solve: (LineSolve3, &'static str), o: &Opts) -> Result<Result3, String> {
    let n = o.n.unwrap_or(case.n);
    let sampler = if o.mem { MemSampler::start() } else { None };
    let ctx = luma::Context::new().map_err(|e| format!("no GPU context: {e}"))?;
    let cfg = case.config(solve.0);
    let mut s = GpuSolver3::new(&ctx, &case.grid, &cfg, precision.0).map_err(|e| format!("{}: {e}", precision.1))?;
    let (first_call_s, us, p, last) = measure(&mut s, n)?;
    if o.profile {
        println!(
            "\n### {} {} {} {} - one implicit step as the eval loop launches it: {} kernel launches, {:.0} us GPU time, {} fusion cuts\n```\n{}```",
            case.name,
            precision.1,
            solve.1,
            case.grid.nk,
            p.launches(),
            p.total().as_secs_f64() * 1e6,
            p.splits.len(),
            compact(&p)
        );
    }
    if !last.is_finite() {
        if !o.allow_nonfinite {
            return Err(format!("{} {} {}: residual {last} after 100 iterations", case.name, precision.1, solve.1));
        }
        eprintln!(
            "WARNING {} {} {}: residual {last} after 100 iterations (--allow-nonfinite: the step time is of a diverging iteration)",
            case.name, precision.1, solve.1
        );
    }
    let peak_mib = sampler.map(MemSampler::stop);
    Ok(Result3 {
        case: case.name.to_string(),
        cells: case.cells(),
        precision: precision.1,
        solve: solve.1,
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
        allow_nonfinite: flag("--allow-nonfinite"),
        n: value("--n").map(|n| n.parse::<usize>().map_err(|e| format!("--n: {e}"))).transpose()?,
    };
    let mut cases = Vec::new();
    for c in if which == "all" { vec!["wedge", "sector", "anf"] } else { vec![which.as_str()] } {
        cases.push(match c {
            "wedge" => wedge_case()?,
            "sector" => sector_case(value("--nk").map_or(Ok(18), |v| v.parse::<usize>().map_err(|e| format!("--nk: {e}")))?)?,
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
                let r = bench_combo(case, precision, solve, &opts)?;
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
                        "PERF {{\"case\":\"3d-{}-euler-{}\",\"precision\":\"{}\",\"cells\":{},\"iters\":{},\"us_per_iter\":{:.3},\"launches_per_iter\":{},\"gpu_us_per_iter\":{:.3},\"wall_s\":null{}}}",
                        case.name,
                        r.solve,
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
    println!("\n| case | cells | precision | line solve | us / iter | launches | GPU us / step | peak MiB (this process) |\n|---|---|---|---|---|---|---|---|");
    for r in &results {
        println!(
            "| {} | {} | {} | {} | {:.0} | {} | {:.0} | {} |",
            r.case,
            r.cells,
            r.precision,
            r.solve,
            r.us,
            r.launches,
            r.gpu_us,
            r.peak_mib.map_or("-".to_string(), |m| m.to_string())
        );
    }
    Ok(())
}
