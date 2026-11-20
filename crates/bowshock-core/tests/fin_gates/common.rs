//! What the fin and incidence gates share (`tests/fin_lift_gates.rs`): the backend (CPU, or the GPU with `BOWSHOCK_WEDGE_BACKEND=gpu` as in
//! `wedge_gates`), the tier (`BOWSHOCK_FIN_TIER=euler|coarse|fine|lift`, default `euler`) and the `GATE` / `INFO` lines.
//!
//! A gate prints `GATE <id> <case> <measured> <reference> <band> PASS|FAIL` when the run is viscous (a wall-resolved tier on an engine that solves the viscous
//! equations) and the same line as `INFO` when it is not: an inviscid result is not compared with data that contains friction. A gate that needs friction
//! (G-F1) fails loudly instead.
#![allow(dead_code)]

use bowshock_cfd::solver3::Solver3;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::run3::{breakdown_of, drag_parts_on, lift_forces, run3_on, DragParts, LiftBreakdown, LiftForces, PointResult3};

/// Which tier the gate runs: the parameter that turns the same test from Euler (today) into RANS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TierKind {
    Euler,
    Coarse,
    Fine,
    Lift,
}

pub fn tier_kind() -> TierKind {
    match std::env::var("BOWSHOCK_FIN_TIER").as_deref() {
        Ok("coarse") => TierKind::Coarse,
        Ok("fine") => TierKind::Fine,
        Ok("lift") => TierKind::Lift,
        Ok("euler") | Err(_) => TierKind::Euler,
        Ok(other) => panic!("BOWSHOCK_FIN_TIER=euler|coarse|fine|lift, got `{other}`"),
    }
}

/// The tier of `kind` for a free stream `flow` (the wall spacing and the fin stack are sized from it; `Euler` ignores it).
pub fn tier(kind: TierKind, flow: WallFlow) -> Tier {
    match kind {
        TierKind::Euler => Tier::Euler,
        TierKind::Coarse => Tier::Coarse(flow),
        TierKind::Fine => Tier::Fine(flow),
        TierKind::Lift => Tier::Lift(flow),
    }
}

/// Does the engine solve the viscous equations for `case`? Both runners do when the case is viscous (`Case3::viscous`: the wall-resolved tiers;
/// `Case3::euler_equations` drops it): the CPU solver and, since `gpu3d` integrated the viscous step, the GPU runner.
pub fn engine_is_viscous(case: &Case3) -> bool {
    case.viscous.is_some()
}

/// Does a run of `kind` count as a gate (viscous) or as INFO (inviscid)?
pub fn is_gate(kind: TierKind, case: &Case3) -> bool {
    kind != TierKind::Euler && engine_is_viscous(case)
}

/// Whether the gates run on the GPU (`BOWSHOCK_WEDGE_BACKEND=gpu`).
///
/// A job of `tools/gpuq` has `GPUQ_JOB` (its id) in its environment, and a gate under it that would take the CPU backend is a lost job, not a slow one: it runs for hours
/// without a GPU process and is killed by the queue's watchdog, or worse, finishes with CPU numbers under a GPU label. So it panics at the first use, with the fix. Without
/// `GPUQ_JOB` (a plain `cargo test`) the CPU backend is the default, as before.
pub fn gpu_backend() -> bool {
    let gpu = std::env::var("BOWSHOCK_WEDGE_BACKEND").as_deref() == Ok("gpu");
    if !gpu {
        if let Some(job) = std::env::var_os("GPUQ_JOB") {
            panic!(
                "gpuq job {job:?} runs a gate on the CPU backend: BOWSHOCK_WEDGE_BACKEND is {:?}, not \"gpu\". Use tools/release_gates.sh, or submit with `env BOWSHOCK_WEDGE_BACKEND=gpu ...`",
                std::env::var("BOWSHOCK_WEDGE_BACKEND").unwrap_or_default()
            );
        }
    }
    gpu
}

/// A Mach list from `BOWSHOCK_GATE_MACHS` (comma list) restricted to the members of `all`; every member when unset.
pub fn machs(all: &[f64]) -> Vec<f64> {
    match std::env::var("BOWSHOCK_GATE_MACHS") {
        Ok(v) => {
            let want: Vec<f64> = v.split(',').map(|x| x.trim().parse().expect("BOWSHOCK_GATE_MACHS: numbers")).collect();
            all.iter().copied().filter(|m| want.iter().any(|w| (w - m).abs() < 1e-9)).collect()
        }
        Err(_) => all.to_vec(),
    }
}

/// What one converged run of a case gives.
pub struct Solved {
    pub point: PointResult3,
    pub parts: DragParts,
    pub lift: LiftForces,
    pub breakdown: LiftBreakdown,
    /// Fin friction force coefficient (on `pi/4`, all fins: `DragParts::fin_friction`), `None` for an inviscid engine.
    pub fin_friction: Option<f64>,
}

impl Solved {
    /// Pressure drag of the fins (sides, edges, tips).
    pub fn fin_pressure(&self) -> f64 {
        self.parts.fin_sides + self.parts.fin_edges + self.parts.fin_tips
    }
    /// Pressure drag of the body without the base (nose and afterbody).
    pub fn body_pressure(&self) -> f64 {
        self.parts.nose + self.parts.aft.iter().sum::<f64>()
    }
    pub fn stop(&self) -> String {
        format!("{} it, {}", self.point.iterations, self.point.stop_reason)
    }
}

/// A solver of one grid on the chosen backend, kept between runs (a warm start from one incidence to the next).
pub enum Session {
    Cpu(Box<Solver3>),
    #[cfg(feature = "gpu")]
    Gpu(Box<bowshock_core::engine::gpu3::GpuRunner3>),
}

impl Session {
    pub fn new(case: &Case3) -> Result<Session, String> {
        #[cfg(feature = "gpu")]
        if gpu_backend() {
            // single precision, the product's (`analysis.rs`): the sector grids are 0.68 M cells, and df32 needs 1.35 x the memory; `BOWSHOCK_WEDGE_PRECISION=ext` for df32
            let prec = if std::env::var("BOWSHOCK_WEDGE_PRECISION").as_deref() == Ok("ext") {
                bowshock_cfd::gpu::Precision::Extended
            } else {
                bowshock_cfd::gpu::Precision::F32
            };
            let r = bowshock_core::engine::gpu3::GpuRunner3::new(case, prec).map_err(|e| e.to_string())?;
            return Ok(Session::Gpu(Box::new(r)));
        }
        Ok(Session::Cpu(Box::new(case.build_solver_cached().map_err(|e| e.to_string())?)))
    }

    /// Runs `case` (same grid as the session's) to its stop rule; `warm` keeps the field of the previous run.
    pub fn run(&mut self, case: &Case3, warm: bool) -> Result<Solved, String> {
        match self {
            Session::Cpu(s) => {
                let point = run3_on(case, s, warm);
                let parts = drag_parts_on(case, s);
                let fin_friction = s.viscous().map(|_| parts.fin_friction());
                Ok(Solved { point, lift: lift_forces(case, s), breakdown: breakdown_of(case, s), parts, fin_friction })
            }
            #[cfg(feature = "gpu")]
            Session::Gpu(r) => {
                let point = r.run_on(case, warm).map_err(|e| e.to_string())?;
                let parts = r.drag_parts(case).map_err(|e| e.to_string())?;
                let fin_friction = r.host.rans().map(|_| parts.fin_friction());
                Ok(Solved {
                    point,
                    parts,
                    lift: r.lift_forces(case).map_err(|e| e.to_string())?,
                    breakdown: r.breakdown(case).map_err(|e| e.to_string())?,
                    fin_friction,
                })
            }
        }
    }
}

/// `f` over `items`: in threads on the CPU (`BOWSHOCK_GATE_THREADS`, default 4; a solver is parallel itself), one after the other on the GPU.
pub fn par_map<T: Sync, R: Send>(items: &[T], f: impl Fn(&T) -> R + Sync) -> Vec<R> {
    let n = if gpu_backend() { 1 } else { std::env::var("BOWSHOCK_GATE_THREADS").ok().and_then(|v| v.parse().ok()).unwrap_or(4usize).max(1) };
    let mut out: Vec<Option<R>> = items.iter().map(|_| None).collect();
    for chunk in items.chunks(n).zip(out.chunks_mut(n)) {
        std::thread::scope(|sc| {
            let hs: Vec<_> = chunk.0.iter().map(|it| sc.spawn(|| f(it))).collect();
            for (h, slot) in hs.into_iter().zip(chunk.1.iter_mut()) {
                *slot = Some(h.join().expect("case thread"));
            }
        });
    }
    out.into_iter().map(|r| r.expect("result")).collect()
}

/// How a measured value is compared with its reference.
#[derive(Clone, Copy, Debug)]
pub enum Band {
    /// `|m - r| <= rel |r|`.
    Rel(f64),
    /// `|m - r| <= max(rel |r|, abs)`.
    RelAbs(f64, f64),
    /// `lo <= m / r <= hi`.
    Ratio(f64, f64),
}

impl Band {
    pub fn pass(&self, m: f64, r: f64) -> bool {
        if !m.is_finite() || !r.is_finite() {
            return false;
        }
        match *self {
            Band::Rel(x) => (m - r).abs() <= x * r.abs(),
            Band::RelAbs(x, a) => (m - r).abs() <= (x * r.abs()).max(a),
            Band::Ratio(lo, hi) => (lo..=hi).contains(&(m / r)),
        }
    }
    pub fn label(&self) -> String {
        match *self {
            Band::Rel(x) => format!("rel{}%", x * 100.0),
            Band::RelAbs(x, a) => format!("rel{}%+abs{a}", x * 100.0),
            Band::Ratio(lo, hi) => format!("ratio[{lo},{hi}]"),
        }
    }
}

fn fmt(x: f64) -> String {
    if x == 0.0 || (1e-3..1e5).contains(&x.abs()) {
        format!("{x:.6}")
    } else {
        format!("{x:.4e}")
    }
}

/// The lines of one test; [`Lines::finish`] fails the test when a `GATE` line failed.
#[derive(Default)]
pub struct Lines {
    pub failed: usize,
    /// Gate lines that fail against a documented discrepancy of the reference ([`Lines::check_known`]): printed `FAIL KNOWN <where>`, not counted in `failed`.
    pub known: usize,
    pub gates: usize,
    pub infos: usize,
}

impl Lines {
    /// A comparison: `GATE ... PASS|FAIL` when `gate`, else `INFO ...` with the verdict it would have had.
    pub fn check(&mut self, gate: bool, id: &str, case: &str, measured: f64, reference: f64, band: Band) -> bool {
        let ok = band.pass(measured, reference);
        if gate {
            println!("GATE {id} {case} {} {} {} {}", fmt(measured), fmt(reference), band.label(), if ok { "PASS" } else { "FAIL" });
            self.gates += 1;
            self.failed += usize::from(!ok);
        } else {
            println!("INFO {id} {case} {} {} {} (inviscid: {})", fmt(measured), fmt(reference), band.label(), if ok { "within" } else { "outside" });
            self.infos += 1;
        }
        ok
    }

    /// [`Lines::check`] for a line whose failure is a documented discrepancy with the reference (`why`: where it is documented, e.g. `DESIGN3D-L-item-37`): a failing
    /// gate prints `GATE ... FAIL KNOWN <why>` and does not fail the test; the band is not widened, and a line that passes prints `PASS` as any other.
    #[allow(clippy::too_many_arguments)]
    pub fn check_known(&mut self, gate: bool, id: &str, case: &str, measured: f64, reference: f64, band: Band, why: &str) -> bool {
        let ok = band.pass(measured, reference);
        if gate && !ok {
            println!("GATE {id} {case} {} {} {} FAIL KNOWN {why}", fmt(measured), fmt(reference), band.label());
            self.gates += 1;
            self.known += 1;
            return false;
        }
        self.check(gate, id, case, measured, reference, band)
    }

    /// A number with no band.
    pub fn info(&mut self, id: &str, case: &str, text: &str) {
        println!("INFO {id} {case} {text}");
        self.infos += 1;
    }

    pub fn finish(self) {
        assert!(self.failed == 0, "{} GATE lines failed", self.failed);
    }
}
