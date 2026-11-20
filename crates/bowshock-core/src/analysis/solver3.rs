//! The [`Engine::Solver3`] route of [`Analysis::run`]: every condition on the 3D solver and the base-wake grid, the one engine the product moves to
//! (`DESIGN3D.md`). Selected by [`Analysis::engine`] or `BOWSHOCK_ENGINE=3d`; [`Engine::Legacy`] stays the default until the viscous 3D solver is
//! validated, and then this file is the route (the legacy drag and lift paths of the parent module and the `engine/{run,gpu,case}.rs` 2D solver are deleted).
//!
//! | condition | case | domain |
//! |---|---|---|
//! | zero incidence, no fins | [`Case3::wedge`] | one cell of [`WEDGE_ANGLE`](crate::engine::grid3::WEDGE_ANGLE), the axisymmetric flow; `Block` line solve (the case sets it) |
//! | zero incidence, fins | [`Case3::sector`] | `[0, pi/n]` between the pitch plane and the fin's mid-plane |
//! | incidence | [`Case3::base_half`] | `[0, pi]`, fins between the pitch plane and each other |
//!
//! Results come from the class forces of the wall faces ([`drag_parts`](crate::engine::run3::drag_parts): nose, each aft segment, base, fin sides, edges
//! and tips; on the GPU reduced on the device), the wall pressures of [`wall_samples`] and the solved state (the guards of [`quality3`]). Coefficients
//! are on the nose-base area the engine works on, rescaled to the reference area in one place ([`Analysis::area_ratio`]).
//!
//! What the engine does not do yet fails loudly: a motor request is an error under [`Engine::Solver3`], never a silent run without the jet. The viscous knobs
//! ([`Analysis::viscous`], roughness, a drag tolerance) map onto the engine's [`Tier`]s through [`Analysis::wall_flow`] and [`Analysis::tier3`]; both backends solve them.

//!
//! # The flip
//!
//! With `BOWSHOCK_ENGINE=3d` the whole app already runs on this engine (`cfd/run.rs` `node_analysis`, `cfd/plan.rs` fin counts and `*_REV_3D` keys, `cfd/wording.rs`
//! `Words(true)`, the shared grid), so the flip is: `Engine::DEFAULT = Solver3` (`analysis.rs`); the app already sends
//! `viscous`, roughness, the drag tolerance and `grid_mach` to both node kinds, and `tier3` maps them; the Jet patch (the motor request here); then delete the legacy
//! path (the route comment in `analysis.rs` lists it), `Words(false)`, `SOLVER_REV` / `LIFT_REV` (the `_3D` numbers take their place) and the app's two-engine branches
//! (`solves_3d`, `shared_grid`).

use super::*;
use crate::engine::axisym_init::{axisym_to_sector, AxisymInit};
use crate::engine::case3::{Tier, WallFlow};
use crate::engine::field3::field3_of;
use crate::engine::grid3::Fins3;
use crate::engine::planform::Planform;
use crate::engine::run3::{drag_parts_on, wall_faces, wall_samples, DragParts, Wall};
use crate::engine::warm3::{rescale, warm_plan, Rescale};
use bowshock_cfd::gas3::{cons_to_prim3, prim_to_cons3, Cons3};
use bowshock_cfd::grid3::{Grid3, PART_FLUID};
use bowshock_cfd::sst::WallThermal;

/// The solved domain of a condition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Wedge,
    Sector,
    Half,
}

/// Where the facade's time of one point went outside the runner's own `PointTiming` (instrumentation only): the case and the run call, what is read back from the
/// state and the teardown of a solver state. Logged per point and read with [`take_facade_timings`]; printed as `TIMING3 {json}` under `BOWSHOCK_TIMING`.
#[derive(Clone, Debug, Default)]
#[doc(hidden)]
pub struct FacadeTiming {
    /// `"point"` or `"drop"` (the teardown of a state).
    pub what: &'static str,
    pub mach: f64,
    pub alpha_deg: f64,
    pub kind: &'static str,
    /// Building the case ([`Analysis::solver3_case`]).
    pub case_s: f64,
    /// The whole run call: `cold` (grid or cache, context, upload, kernels, iterations) or `warm`.
    pub run_s: f64,
    /// [`Runner3::post`]: the device force reduction and the state download.
    pub post_s: f64,
    /// The guards, the wall samples and the point's numbers on the host.
    pub host_s: f64,
    /// The final `Field3`.
    pub field_s: f64,
    /// Dropping a solver state (device arrays, kernels).
    pub drop_s: f64,
    /// The cold run was on a solver kept from an earlier point (see `POOL`).
    pub reused: bool,
    /// The axisymmetric seed of the point (see [`Analysis::seed_axisymmetric`]): `""` none (wedge point, no fins, opted out, or a warm run), `"seeded"` the run started from the
    /// wedge, `"no-seed"` the wedge failed and the run started cold, `"rerun"` the seeded run ended `Failed` or `Stalled` and was run again from the free stream.
    pub seed: &'static str,
    /// Wall seconds and iterations of the seed wedge solved for this point (zero when the wedge of its Mach number and tier was solved for an earlier point of the sweep).
    pub seed_s: f64,
    pub seed_iters: usize,
    /// The start from the converged field of a neighbouring Mach number ([`Analysis::warm_start`]): `""` none, `"warm"` the run started from it and settled, `"rerun"` it ended
    /// `Failed` or `Stalled` and the point was run again from its seed or the free stream.
    pub neighbour: &'static str,
}

static FACADE_TIMINGS: std::sync::Mutex<Vec<FacadeTiming>> = std::sync::Mutex::new(Vec::new());

fn record_facade(t: FacadeTiming) {
    if std::env::var_os("BOWSHOCK_TIMING").is_some() {
        eprintln!(
            "TIMING3 {{\"what\":\"{}\",\"kind\":\"{}\",\"mach\":{},\"alpha_deg\":{:.1},\"case_s\":{:.4},\"run_s\":{:.4},\"post_s\":{:.4},\"host_s\":{:.4},\"field_s\":{:.4},\"drop_s\":{:.4},\"reused\":{},\"seed\":\"{}\",\"seed_s\":{:.4},\"seed_iters\":{}}}",
            t.what, t.kind, t.mach, t.alpha_deg, t.case_s, t.run_s, t.post_s, t.host_s, t.field_s, t.drop_s, t.reused, t.seed, t.seed_s, t.seed_iters
        );
    }
    FACADE_TIMINGS.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push(t);
}

/// Drains the facade's timing log (see [`FacadeTiming`]).
#[doc(hidden)]
pub fn take_facade_timings() -> Vec<FacadeTiming> {
    std::mem::take(&mut *FACADE_TIMINGS.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}

/// Hands `state` back to the runner (dropped or kept), logging how long that took.
fn drop_timed<R: Runner3>(runner: &R, state: Option<R::State>, c: Condition) {
    if let Some(s) = state {
        let t = std::time::Instant::now();
        runner.recycle(s);
        record_facade(FacadeTiming { what: "drop", mach: c.mach, alpha_deg: c.alpha_deg, drop_s: t.elapsed().as_secs_f64(), ..Default::default() });
    }
}

/// What a backend hands back about a finished run: the class forces, the state and its grid.
struct Post3 {
    drag: DragParts,
    u: Vec<Cons3>,
    /// Eddy over molecular viscosity per cell, and the skin-friction coefficient vector of every wall face of `wall_faces` (empty unless RANS on the CPU).
    eddy: Vec<f32>,
    cf: Vec<[f32; 3]>,
}

/// The eddy-viscosity ratio of every cell and the skin-friction coefficient (over the free stream's `rho |V|^2 / 2`) of every wall face of a RANS state; empty for
/// Euler. The faces are in the order of [`wall_faces`], which is the order of `Field3::walls`.
fn viscous_extras(case: &Case3, s: &Solver3) -> (Vec<f32>, Vec<[f32; 3]>) {
    let Some(r) = s.rans() else { return (Vec::new(), Vec::new()) };
    let ratio: Vec<f64> = r.mu_t.iter().zip(&r.mu).map(|(t, m)| t / m).collect();
    extras_of(case, &s.grid, &ratio, &s.wall_tractions())
}

/// [`viscous_extras`] from the eddy ratio per cell and the tractions of the no-slip faces.
fn extras_of(case: &Case3, g: &Grid3, ratio: &[f64], tractions: &[bowshock_cfd::viscous3::WallTraction]) -> (Vec<f32>, Vec<[f32; 3]>) {
    let eddy = ratio.iter().map(|&r| r as f32).collect();
    let dir_of = |c: [(usize, usize, usize); 4]| {
        if c.iter().all(|n| n.0 == c[0].0) {
            0
        } else if c.iter().all(|n| n.1 == c[0].1) {
            1
        } else {
            2
        }
    };
    let faces = wall_faces(g);
    let index: std::collections::HashMap<(usize, usize, usize, usize), usize> =
        faces.iter().enumerate().map(|(n, w)| ((dir_of(w.corners), w.corners[0].0, w.corners[0].1, w.corners[0].2), n)).collect();
    let q = 0.5 * case.mach * case.mach;
    let mut cf = vec![[0.0f32; 3]; faces.len()];
    for t in tractions {
        if let Some(&n) = index.get(&(t.dir, t.idx[0], t.idx[1], t.idx[2])) {
            cf[n] = t.tau.map(|v| (v / q) as f32);
        }
    }
    (eddy, cf)
}

/// The most iterations of a seed wedge: it only has to settle the base wake roughly, the finned run takes it from there.
const SEED_MAX_ITERATIONS: usize = 3000;

/// Factor on [`crate::engine::case3::Numerics3::drift_tol`] of a seeded run.
const SEED_DRIFT_TOL_FACTOR: f64 = 0.5;

/// The axisymmetric solution a finned sector starts from: the wedge of its body, Mach number and tier (see [`Analysis::seed_axisymmetric`]).
struct Seed {
    grid: Grid3,
    u: Vec<Cons3>,
    /// `[k, omega]` of every wedge cell (`None`: Euler).
    turb: Option<Vec<[f64; 2]>>,
    iterations: usize,
}

impl Seed {
    /// The seed of a finished wedge run: `None` unless it ended without failing (a run capped at [`SEED_MAX_ITERATIONS`] is a seed) and every fluid cell holds a finite
    /// state of positive density and pressure.
    fn of(r: &PointResult3, grid: Grid3, u: Vec<Cons3>, turb: Option<Vec<[f64; 2]>>) -> Option<Seed> {
        let fine = field_is_sound(&grid.part, &u, turb.as_deref());
        (lift_quality(r) != Quality::Failed && fine).then_some(Seed { grid, u, turb, iterations: r.iterations })
    }

    /// This field carried to the fluid cells of `dst` ([`axisym_to_sector`]: the nearest meridian cell, the velocity rotated to the azimuth of the cell, which holds for any
    /// azimuth range of `dst`: the sector, and the half domain with its pitch plane at `theta = 0` and `pi`); the other cells keep the free stream of `case`.
    fn on(&self, case: &Case3, dst: &Grid3) -> AxisymInit {
        let fs = prim_to_cons3(&case.freestream());
        let turb_fs = case.viscous.map_or([0.0; 2], |v| [bowshock_cfd::sst::K_INF, bowshock_cfd::sst::omega_inf(v.re_c)]);
        // an Euler seed has no `k`, `omega`: the transfer takes zeros and its `turb` is not used
        let none;
        let src_turb = match &self.turb {
            Some(t) => t.as_slice(),
            None => {
                none = vec![[0.0; 2]; self.u.len()];
                &none
            }
        };
        axisym_to_sector(&self.grid, &self.u, src_turb, dst, fs, turb_fs)
    }
}

/// Whether every fluid cell of `u` holds a finite state of positive density and pressure and `turb` (when given) is finite.
fn field_is_sound(part: &[u8], u: &[Cons3], turb: Option<&[[f64; 2]]>) -> bool {
    let sound = |c: usize| {
        let w = cons_to_prim3(&u[c]);
        w.iter().all(|x| x.is_finite()) && w[0] > 0.0 && w[4] > 0.0
    };
    (0..u.len()).filter(|&c| part[c] == PART_FLUID).all(sound) && turb.is_none_or(|t| t.iter().all(|x| x.iter().all(|v| v.is_finite())))
}

/// How the sweep of [`Analysis::warm_start`] is run: the order of its points and how a field is carried from one to the next. Fixed by measurement on the finned example
/// (`DESIGN3D.md` item 59); the `BOWSHOCK_WARM_*` variables override them for the measurements only.
#[derive(Clone, Copy, Debug)]
struct WarmTuning {
    /// The points above this Mach number are solved from the highest one downward, the others from the lowest upward (`sub_up`) or downward.
    split: f64,
    sub_up: bool,
    /// A neighbour further than this factor in Mach number is not a start (see [`warm_plan`]).
    max_ratio: f64,
    /// The points above this Mach number start as they do without the flag (see [`warm_plan`]).
    max_mach: f64,
    rescale: Rescale,
    /// Factor on [`crate::engine::case3::Numerics3::drift_tol`] of a run started from a neighbour: it starts close to the answer, like a seeded run.
    drift_factor: f64,
    /// A warm run goes on at the CFL the neighbour's run ended at (its `cfl_max` after the controller's answers) instead of restarting the ramp at `cfl0`.
    keep_cfl: bool,
}

/// The runs of a warm sweep over the Mach numbers `machs` ([`crate::engine::warm3::warm_runs`] with the sweep's settings): the indices of the points that start from one
/// another, ascending, each run solved in one `run` call on one machine; the app makes a work item of each.
#[doc(hidden)]
pub fn warm_runs(machs: &[f64]) -> Vec<Vec<usize>> {
    let t = WarmTuning::current();
    crate::engine::warm3::warm_runs(machs, t.split, t.sub_up, t.max_ratio, t.max_mach)
}

impl WarmTuning {
    fn current() -> WarmTuning {
        let var = |name: &str| std::env::var(name).ok();
        let num = |name: &str, default: f64| var(name).and_then(|v| v.parse().ok()).unwrap_or(default);
        WarmTuning {
            split: num("BOWSHOCK_WARM_SPLIT", 0.9),
            sub_up: var("BOWSHOCK_WARM_SUB").is_none_or(|v| v != "down"),
            max_ratio: num("BOWSHOCK_WARM_RATIO", 10.0),
            max_mach: num("BOWSHOCK_WARM_MAXMACH", 10.0),
            rescale: match var("BOWSHOCK_WARM_RESCALE").as_deref() {
                Some("none") => Rescale::None,
                Some("velocity") => Rescale::Velocity,
                _ => Rescale::Perturbation,
            },
            drift_factor: num("BOWSHOCK_WARM_DRIFT", SEED_DRIFT_TOL_FACTOR),
            keep_cfl: var("BOWSHOCK_WARM_RAMP").is_none_or(|v| v != "restart"),
        }
    }
}

/// The converged field of a point of a warm sweep, host side: what its neighbours of the sweep start from.
struct Carried {
    u: Vec<Cons3>,
    /// `[k, omega]` of every cell (`None`: Euler).
    turb: Option<Vec<[f64; 2]>>,
    mach: f64,
    alpha: f64,
    /// The `cfl_max` its run ended with (the controller's answers included).
    cfl: f64,
    /// The grid it is on: a field only starts a point on the same grid.
    grid: crate::engine::cache3::GridKey,
}

impl Carried {
    /// The field of a finished point (`None` unless it holds a sound state everywhere).
    fn of(case: &Case3, grid: &Grid3, u: Vec<Cons3>, turb: Option<Vec<[f64; 2]>>, cfl: f64) -> Option<Carried> {
        field_is_sound(&grid.part, &u, turb.as_deref()).then(|| Carried {
            u,
            turb,
            mach: case.mach,
            alpha: case.alpha,
            cfl,
            grid: crate::engine::cache3::GridKey::of(&case.grid),
        })
    }

    /// Whether this field can start `case`: the same grid and incidence.
    fn fits(&self, case: &Case3) -> bool {
        self.grid == crate::engine::cache3::GridKey::of(&case.grid) && self.alpha == case.alpha
    }
}

/// What a run that does not start from the free stream starts from.
enum Start<'a> {
    /// The axisymmetric wedge solution of the same Mach number.
    Seed(&'a Seed),
    /// The field of a neighbouring Mach number, carried to that of the case by `Rescale`.
    Neighbour {
        from: &'a Carried,
        rescale: Rescale,
        /// The CFL the run goes on at (`None`: the ramp restarts at `cfl0`).
        cfl: Option<f64>,
    },
}

impl Start<'_> {
    /// The state and the `[k, omega]` (`None` for Euler) of every cell of `dst`, the grid `case` was built on.
    fn on(&self, case: &Case3, dst: &Grid3) -> (Vec<Cons3>, Option<Vec<[f64; 2]>>) {
        match self {
            Start::Seed(seed) => {
                let m = seed.on(case, dst);
                (m.u, seed.turb.is_some().then_some(m.turb))
            }
            Start::Neighbour { from, rescale: mode, .. } => (rescale(&from.u, &dst.part, from.mach, case.mach, case.alpha, *mode), from.turb.clone()),
        }
    }
}

/// One backend of the sweep: a cold run that builds the solver state, a warm run on it, and what is read from a state afterwards.
trait Runner3 {
    type State;
    fn cold(&self, case: &Case3) -> Result<(PointResult3, Self::State), Error>;
    /// A cold run that starts from `start` instead of the free stream (the axisymmetric seed, or the field of a neighbouring Mach number): the field carried to the grid of
    /// `case` and a warm run on it (the first-order start-up skipped, the CFL ramp restarted).
    fn cold_from(&self, case: &Case3, start: &Start) -> Result<(PointResult3, Self::State), Error>;
    /// A warm run on `state`; `cfl` is the CFL it goes on at (`None`: the ramp restarts; the CPU runner always restarts it).
    fn warm(&self, state: &mut Self::State, case: &Case3, cfl: Option<f64>) -> Result<PointResult3, Error>;
    /// Solves the wedge `case` to its stop rule and returns its field; `None` when it failed.
    fn seed(&self, wedge: &Case3) -> Result<Option<Seed>, Error>;
    fn grid<'a>(&self, state: &'a Self::State) -> &'a Grid3;
    /// The class forces and the state of `state` (the one download of a point).
    fn post(&self, state: &Self::State, case: &Case3) -> Result<Post3, Error>;
    /// The finished state of `state` as the field a neighbouring point starts from (`None` when it is not sound).
    fn carry(&self, state: &Self::State, case: &Case3, cfl: f64) -> Result<Option<Carried>, Error>;
    /// Whether `state` was built for an earlier point and kept (see [`Runner3::recycle`]).
    fn reused(&self, _state: &Self::State) -> bool {
        false
    }
    /// A state that is no longer needed: dropped, or kept for the next cold run on the same grid.
    fn recycle(&self, state: Self::State) {
        drop(state);
    }
}

struct Cpu3;

impl Runner3 for Cpu3 {
    type State = Solver3;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, Solver3), Error> {
        let mut s = case.build_solver_cached()?;
        let r = run3_on(case, &mut s, false);
        Ok((r, s))
    }

    fn cold_from(&self, case: &Case3, start: &Start) -> Result<(PointResult3, Solver3), Error> {
        let mut s = case.build_solver_cached()?;
        let (u, turb) = start.on(case, &s.grid);
        s.set_state_turb(&u, turb.as_deref());
        let r = run3_on(case, &mut s, true);
        Ok((r, s))
    }

    fn warm(&self, s: &mut Solver3, case: &Case3, _cfl: Option<f64>) -> Result<PointResult3, Error> {
        Ok(run3_on(case, s, true))
    }

    fn seed(&self, wedge: &Case3) -> Result<Option<Seed>, Error> {
        let mut s = wedge.build_solver_cached()?;
        let r = run3_on(wedge, &mut s, false);
        if r.stop_reason == StopReason::Cancelled {
            return Err(Error::Cancelled);
        }
        let turb = s.rans().map(|r| r.turb.clone()).filter(|t| !t.is_empty());
        Ok(Seed::of(&r, s.grid.clone(), s.u.clone(), turb))
    }

    fn grid<'a>(&self, s: &'a Solver3) -> &'a Grid3 {
        &s.grid
    }

    fn post(&self, s: &Solver3, case: &Case3) -> Result<Post3, Error> {
        let (eddy, cf) = viscous_extras(case, s);
        Ok(Post3 { drag: drag_parts_on(case, s), u: s.u.clone(), eddy, cf })
    }

    fn carry(&self, s: &Solver3, case: &Case3, cfl: f64) -> Result<Option<Carried>, Error> {
        let turb = s.rans().map(|r| r.turb.clone()).filter(|t| !t.is_empty());
        Ok(Carried::of(case, &s.grid, s.u.clone(), turb, cfl))
    }
}

/// A GPU solver with the grid and precision it was built for.
#[cfg(feature = "gpu")]
struct GpuState {
    runner: crate::engine::gpu3::GpuRunner3,
    key: (crate::engine::cache3::GridKey, bool),
    reused: bool,
}

#[cfg(feature = "gpu")]
thread_local! {
    /// The GPU solver of the last point of this thread, kept for the next cold run on the same grid: the device arrays, the uploaded geometry and the traced
    /// kernels are the cost of a point that does not depend on its Mach number (a cold run resets the state to the free stream of its case). The fleet's machine
    /// thread solves the nodes of a table one after the other, one `run` call each.
    static POOL: std::cell::RefCell<Option<GpuState>> = const { std::cell::RefCell::new(None) };
}

#[cfg(feature = "gpu")]
struct Gpu3;

#[cfg(feature = "gpu")]
impl Runner3 for Gpu3 {
    type State = GpuState;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, GpuState), Error> {
        // The wedge (a few thousand cells, solved to a tight stop rule at low Mach number too) runs in extended precision as the axisymmetric solver did; the half
        // domain and the sector (hundreds of thousands of cells, within 0.01 % of extended on the ANF table) in single precision.
        self.cold_in(case, matches!(case.grid.domain, crate::engine::grid3::Domain3::Wedge { .. }), None)
    }

    fn cold_from(&self, case: &Case3, start: &Start) -> Result<(PointResult3, GpuState), Error> {
        self.cold_in(case, matches!(case.grid.domain, crate::engine::grid3::Domain3::Wedge { .. }), Some(start))
    }

    fn warm(&self, r: &mut GpuState, case: &Case3, cfl: Option<f64>) -> Result<PointResult3, Error> {
        r.runner.start_cfl = cfl;
        r.runner.run_on(case, true)
    }

    fn seed(&self, wedge: &Case3) -> Result<Option<Seed>, Error> {
        // single precision like the finned run it seeds (the wedge's own product runs stay extended: `Gpu3::cold`); the solver is dropped on return
        let mut runner = crate::engine::gpu3::GpuRunner3::new(wedge, bowshock_cfd::gpu::Precision::F32)?;
        let r = runner.run_on(wedge, false)?;
        if r.stop_reason == StopReason::Cancelled {
            return Err(Error::Cancelled);
        }
        let (u, turb) = (runner.gpu.state_f64()?, runner.gpu.turb_state()?.map(|(t, _)| t));
        Ok(Seed::of(&r, runner.host.grid.clone(), u, turb))
    }

    fn grid<'a>(&self, r: &'a GpuState) -> &'a Grid3 {
        &r.runner.host.grid
    }

    fn post(&self, r: &GpuState, case: &Case3) -> Result<Post3, Error> {
        let u = r.runner.gpu.state_f64()?;
        let (mut eddy, mut cf) = (Vec::new(), Vec::new());
        // a viscous run keeps its field's extras: the eddy ratio from the device's `mu_t` and the molecular viscosity of the device state, the skin friction from the
        // device's viscous face fluxes (which the wall forces were reduced from)
        if let (Some(host), Some((_, mu_t)), Some(fluxes)) = (r.runner.host.rans(), r.runner.gpu.turb_state()?, r.runner.gpu.viscous_flux()?) {
            let g = &r.runner.host.grid;
            let ratio: Vec<f64> = u
                .iter()
                .zip(&mu_t)
                .map(|(c, t)| {
                    let w = bowshock_cfd::gas3::cons_to_prim3(c);
                    t / host.visc.mu(bowshock_cfd::gas3::GAMMA * w[4] / w[0])
                })
                .collect();
            let solid: Vec<bool> = g.part.iter().map(|&p| p != bowshock_cfd::grid3::PART_FLUID).collect();
            let tractions = bowshock_cfd::viscous3::wall_tractions(g, &solid, &host.dirs, &fluxes[0], &fluxes[1], &fluxes[2]);
            (eddy, cf) = extras_of(case, g, &ratio, &tractions);
        }
        Ok(Post3 { drag: r.runner.drag_parts(case)?, u, eddy, cf })
    }

    fn carry(&self, r: &GpuState, case: &Case3, cfl: f64) -> Result<Option<Carried>, Error> {
        let (u, turb) = (r.runner.gpu.state_f64()?, r.runner.gpu.turb_state()?.map(|(t, _)| t));
        Ok(Carried::of(case, &r.runner.host.grid, u, turb, cfl))
    }

    fn reused(&self, r: &GpuState) -> bool {
        r.reused
    }

    fn recycle(&self, state: GpuState) {
        // Opt-in (`BOWSHOCK_RUNNER_REUSE=1`) until it is shown to change nothing: the fixed cost it saves is under 4 % of a table (see the measurements in the report of
        // the engine timing), and a pooled finned half domain diverged at points a fresh solver does not.
        if std::env::var_os("BOWSHOCK_RUNNER_REUSE").is_some() {
            POOL.with(|p| *p.borrow_mut() = Some(state));
        }
    }
}

#[cfg(feature = "gpu")]
impl Gpu3 {
    /// A cold run in extended (`extended`) or single precision, on the pooled solver of this thread when it was built for the same grid and precision; from the free stream, or
    /// (`start`) from the field of an axisymmetric wedge or of a neighbouring Mach number.
    fn cold_in(&self, case: &Case3, extended: bool, start: Option<&Start>) -> Result<(PointResult3, GpuState), Error> {
        let key = (crate::engine::cache3::GridKey::of(&case.grid), extended);
        let kept = POOL.with(|p| p.borrow_mut().take()).filter(|s| s.key == key);
        let mut state = match kept {
            Some(s) => GpuState { reused: true, ..s },
            None => {
                let precision = if extended { bowshock_cfd::gpu::Precision::Extended } else { bowshock_cfd::gpu::Precision::F32 };
                GpuState { runner: crate::engine::gpu3::GpuRunner3::new(case, precision)?, key, reused: false }
            }
        };
        let p = match start {
            Some(start) => {
                let (u, turb) = start.on(case, &state.runner.host.grid);
                state.runner.gpu.set_state_turb(&u, turb.as_deref())?;
                if let Start::Neighbour { cfl, .. } = start {
                    state.runner.start_cfl = *cfl;
                }
                state.runner.run_on(case, true)?
            }
            None => state.runner.run_on(case, false)?,
        };
        Ok((p, state))
    }
}

/// Quality of a 3D point, first match wins: [`Quality::Failed`] on a non-finite or blown-up state (the run stopped `NonFinite`), a cell whose density or
/// pressure is not positive, a non-finite force or drag, a pressure at the outer boundaries more than 100 % off the free stream, or an area-mean base Cp
/// below the vacuum limit `-2 / (gamma M^2)`: the guards of the 2D facade's `quality`. Otherwise by how the run stopped.
#[allow(clippy::too_many_arguments)]
fn quality3(stop: StopReason, mach: f64, min_rho: f64, min_p: f64, finite: bool, farfield_dp: f64, cp_base: f64) -> Quality {
    let below_vacuum = cp_base < -2.0 / (GAMMA * mach * mach); // false for NaN
    let positive = |x: f64| x > 0.0; // false for NaN
    if matches!(stop, StopReason::NonFinite | StopReason::Cancelled)
        || !positive(min_rho)
        || !positive(min_p)
        || !finite
        || farfield_dp > FARFIELD_DP_FAILED
        || below_vacuum
    {
        return Quality::Failed;
    }
    match stop {
        StopReason::ResidualDrop => Quality::Converged,
        StopReason::CdStationary | StopReason::MeanStationary | StopReason::Settled | StopReason::BasePressureStationary | StopReason::ResidualPlateau => {
            Quality::Stationary
        }
        StopReason::MaxIterations => Quality::Stalled,
        StopReason::NonFinite | StopReason::Cancelled => Quality::Failed,
    }
}

impl Analysis {
    /// Which domain solves `c`.
    fn kind_of(&self, c: Condition) -> Kind {
        if c.alpha_deg != 0.0 {
            Kind::Half
        } else if self.fins.is_some() {
            Kind::Sector
        } else {
            Kind::Wedge
        }
    }

    /// Ratio of the engine's nose-base area to the reference area: the one factor that takes the engine's coefficients to the facade's.
    fn area_ratio(&self) -> f64 {
        (self.d_nose / self.d_ref).powi(2)
    }

    /// The input checks of the route.
    fn check_solver3(&self, conditions: &[Condition]) -> Result<(), Error> {
        // the jet patch of the 3D solver does not exist yet: a motor-on request fails whatever else the engine solves
        if self.motor.is_some() && self.flight.is_some() {
            return Err(Error::Unsupported("Engine::Solver3 has no motor jet yet".into()));
        }
        if self.flight.is_none() {
            if self.drag_tolerance.is_some() {
                return Err(Error::InvalidInput("drag_tolerance needs Analysis::viscous".into()));
            }
            if self.roughness_m > 0.0 {
                return Err(Error::InvalidInput("roughness needs Analysis::viscous and a dimensional BodySpec".into()));
            }
            if self.motor.is_some() {
                return Err(Error::InvalidInput("motor-on analysis needs Analysis::viscous".into()));
            }
        }
        for c in conditions {
            let (range, what) = if c.alpha_deg == 0.0 { (MACH_RANGE, "") } else { (LIFT_MACH_RANGE, " for a condition with an angle of attack") };
            if !(c.mach.is_finite() && (range.0..=range.1).contains(&c.mach)) {
                return Err(Error::InvalidInput(format!("Mach number {} is outside [{}, {}]{what}", c.mach, range.0, range.1)));
            }
            if c.alpha_deg != 0.0 && !(c.alpha_deg > 0.0 && c.alpha_deg <= LIFT_ALPHA_MAX_DEG) {
                return Err(Error::InvalidInput(format!("incidence {} deg is outside (0, {LIFT_ALPHA_MAX_DEG}]", c.alpha_deg)));
            }
        }
        if let Some(m) = self.grid_mach {
            if !(m.is_finite() && (MACH_RANGE.0..=MACH_RANGE.1).contains(&m)) {
                return Err(Error::InvalidInput(format!("grid Mach number {m} is outside [{}, {}]", MACH_RANGE.0, MACH_RANGE.1)));
            }
        }
        if !(self.d_ref.is_finite() && self.d_ref > 0.0) {
            return Err(Error::InvalidInput(format!("reference diameter {} must be finite and > 0", self.d_ref)));
        }
        if !(self.roughness_m.is_finite() && self.roughness_m >= 0.0 && self.roughness_m < 0.01 * self.d_nose) {
            return Err(Error::InvalidInput(format!(
                "roughness {} m must be finite, >= 0 and below 1% of the nose diameter ({} m)",
                self.roughness_m,
                0.01 * self.d_nose
            )));
        }
        if let Some(f) = &self.fins {
            f.validate_for(Engine::Solver3)?;
            self.fins3_solver3()?;
            if !self.dimensional {
                return Err(Error::InvalidInput("fins need a dimensional BodySpec".into()));
            }
        }
        Ok(())
    }

    /// The fins as the 3D engine takes them: the trapezoid of the spec, or its outline ([`Analysis::fin_outline`]) through the engine's [`Planform`].
    fn fins3_solver3(&self) -> Result<Option<Fins3>, Error> {
        let Some(f3) = self.fins3() else { return Ok(None) };
        let (d, spec) = (self.d_nose, self.fins.as_ref().expect("fins3 has fins"));
        Ok(Some(match &self.fin_outline {
            FinOutline::Trapezoid => f3,
            FinOutline::Ellipse => f3.with_planform(Planform::ellipse(spec.root_chord / d, spec.span / d)?),
            FinOutline::Polygon(points) => {
                let scaled: Vec<(f64, f64)> = points.iter().map(|p| (p.0 / d, p.1 / d)).collect();
                f3.with_planform(Planform::from_polygon(&scaled)?)
            }
        }))
    }

    /// Whether the mesher can build the fins of this analysis (on the coarse Euler sector, which the solve then finds in the grid cache): the mesher's refusal, with its
    /// cause, as an error. The app asks when it maps a document's fin set.
    #[doc(hidden)]
    pub fn mesh_check(&self) -> Result<(), Error> {
        self.check_solver3(&[Condition::from(2.0)])?;
        if self.fins.is_none() {
            return Ok(());
        }
        let case = self.solver3_case(Condition::from(2.0), 0, Tier::Euler, false);
        case.build_solver_cached().map(|_| ())
    }

    /// The flow the walls of a wall-resolved grid are sized for: the viscous knobs of the analysis (`viscous`, `roughness`, the fins' roughness, `grid_mach`)
    /// as the engine's [`WallFlow`]; `None` for an Euler analysis.
    fn wall_flow(&self, machs: &[f64]) -> Result<Option<WallFlow>, Error> {
        let Some(rans) = self.rans(machs)? else {
            return Ok(None);
        };
        Ok(Some(WallFlow {
            re_c: rans.re_c,
            mach_max: rans.mach_max,
            ks_body: self.roughness_m / self.d_nose,
            ks_fin: self.fins.as_ref().map_or(0.0, |f| f.roughness) / self.d_nose,
            sutherland: rans.sutherland,
        }))
    }

    /// The engine's tier of condition `c`: Euler without wall flow; with it the lift tier at an angle of attack and, at zero incidence, `Coarse` for
    /// [`Resolution::Coarse`], `Draft` for [`Resolution::Draft`] and `Fine` for the finer resolutions.
    fn tier3(c: Condition, resolution: Resolution, flow: Option<WallFlow>) -> Tier {
        match flow {
            None => Tier::Euler,
            Some(f) if c.alpha_deg != 0.0 => Tier::Lift(f),
            Some(f) if resolution == Resolution::Draft => Tier::draft(f),
            Some(f) if resolution == Resolution::Coarse => Tier::Coarse(f),
            Some(f) => Tier::Fine(f),
        }
    }

    /// The case of condition `c` at grid `level`, with the numerics overrides, progress, snapshots and the test hook applied.
    fn solver3_case(&self, c: Condition, level: u32, tier: Tier, snapshots: bool) -> Case3 {
        self.case_of(self.kind_of(c), c, level, tier, snapshots)
    }

    /// The case of `c` on the domain `kind` (the one of the condition, or the wedge of a seed).
    fn case_of(&self, kind: Kind, c: Condition, level: u32, tier: Tier, snapshots: bool) -> Case3 {
        let body = self.body.clone();
        let mut case = match kind {
            Kind::Wedge => Case3::wedge(c.mach, body, level).with_tier(tier),
            Kind::Sector => Case3::sector(c.mach, body, self.fins3_solver3().expect("checked").expect("a sector has fins"), tier, level),
            Kind::Half => Case3::base_half(c.mach, c.alpha_deg.to_radians(), body, self.fins3_solver3().expect("checked"), tier, level),
        };
        if let Some(t) = self.wall_temperature_ratio {
            case.wall_thermal = WallThermal::Isothermal(t);
        }
        if self.wake == Some(Wake::Sting) {
            case.grid.base = None;
        }
        if let Some(n) = &self.numerics {
            case.numerics.limiter = n.limiter;
            case.numerics.low_mach = n.low_mach;
            case.numerics.first_order_iters = n.first_order_iters;
            case.numerics.max_iter = n.max_iter;
        }
        if let Some(cap) = self.max_iterations {
            case.numerics.max_iter = case.numerics.max_iter.min(cap);
        }
        if let Some(tol) = self.drag_tolerance {
            case.numerics.rel_tol = tol;
        }
        case.progress = self.progress.as_ref().map(|p| p.scaled(self.area_ratio()));
        case.snapshots = if snapshots { self.field3_sink.clone() } else { None };
        if let Some(hook) = self.case3_hook {
            hook(&mut case);
        }
        case
    }

    /// The axisymmetric counterpart of the finned case `case` of `c`: the wedge of its body, Mach number and tier (`case.tier`: the meridian of the wall-resolved tiers is the
    /// same), with the numerics overrides and the test hook of the analysis, silent (no progress, no snapshots) and capped at [`SEED_MAX_ITERATIONS`].
    fn seed_case(&self, case: &Case3, c: Condition, level: u32) -> Case3 {
        let mut wedge = self.case_of(Kind::Wedge, c, level, case.tier, false);
        wedge.progress = None;
        wedge.numerics.max_iter = wedge.numerics.max_iter.min(SEED_MAX_ITERATIONS);
        wedge
    }

    /// Whether the cold runs of `kind` start from the axisymmetric wedge: the finned sector at zero incidence, unless [`Analysis::seed_axisymmetric`] is off (or `BOWSHOCK_NO_SEED`
    /// is set, the switch of the timing runs that compare against the plain cold start). The half domain at incidence is not seeded: measured on the Draft example (GPU f32,
    /// DDADI) its seeded points took 1.4 to 1.9 times the iterations of the cold ones (Mach 0.5 1570 against 810, 1.1 870 against 610, 3.0 1170 against 810) and each paid its own wedge.
    fn seeded(&self, kind: Kind) -> bool {
        self.seed_axisymmetric && self.fins.is_some() && kind == Kind::Sector && std::env::var_os("BOWSHOCK_NO_SEED").is_none()
    }

    /// The first run of `case` (condition `c` on `kind`) of a point that has no live solver: from the converged field of a neighbouring Mach number (`neighbour`, a sweep with
    /// [`Analysis::warm_start`]), or from the free stream, or from the wedge seed of its Mach number and tier ([`Analysis::seeded`]; `seeds` keeps the wedges of the sweep, `None`
    /// for one that failed). A run from a neighbour or a seed that ends `Failed` or `Stalled` is run again from the next start (the seed, the free stream), and a wedge that fails
    /// gives a plain cold start.
    #[allow(clippy::too_many_arguments)]
    fn cold_point<R: Runner3>(
        &self,
        runner: &R,
        case: &Case3,
        c: Condition,
        kind: Kind,
        level: u32,
        seeds: &mut std::collections::HashMap<String, Option<Seed>>,
        neighbour: Option<(&Carried, &WarmTuning)>,
        tm: &mut FacadeTiming,
    ) -> Result<(PointResult3, R::State), Error> {
        if let Some((from, tune)) = neighbour.filter(|(from, _)| from.fits(case)) {
            // judged as tightly as a seeded run: it starts close to the answer, and the first geometric tail of the slow base-wake adjustment must not stop it
            let mut warm = case.clone();
            warm.numerics.drift_tol *= tune.drift_factor;
            let start = Start::Neighbour { from, rescale: tune.rescale, cfl: tune.keep_cfl.then_some(from.cfl) };
            let (r, s) = runner.cold_from(&warm, &start)?;
            if r.stop_reason == StopReason::Cancelled || matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
                tm.neighbour = "warm";
                return Ok((r, s));
            }
            tm.neighbour = "rerun";
            runner.recycle(s);
        }
        if !self.seeded(kind) {
            return runner.cold(case);
        }
        let key = format!("{:016x}|{:?}", c.mach.to_bits(), case.tier);
        if !seeds.contains_key(&key) {
            let t = std::time::Instant::now();
            let seed = runner.seed(&self.seed_case(case, c, level))?;
            tm.seed_s = t.elapsed().as_secs_f64();
            tm.seed_iters = seed.as_ref().map_or(0, |s| s.iterations);
            seeds.insert(key.clone(), seed);
        }
        let Some(seed) = &seeds[&key] else {
            tm.seed = "no-seed";
            return runner.cold(case);
        };
        // a seeded run is judged twice as tightly by the extrapolated-drift rule: it starts close to the answer and the rule would stop it on the first geometric tail of the
        // slow base-wake adjustment (Draft example sector, Mach 0.8-1.1: 0.5-1.1 % high with the tolerance of the cold runs, which reach their stop 900-1000 iterations later)
        let mut seeded = case.clone();
        seeded.numerics.drift_tol *= SEED_DRIFT_TOL_FACTOR;
        let (r, s) = runner.cold_from(&seeded, &Start::Seed(seed))?;
        if r.stop_reason == StopReason::Cancelled || matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
            tm.seed = "seeded";
            return Ok((r, s));
        }
        // the seeded run did not settle: the plain cold run decides the point (the failed state is released first)
        tm.seed = "rerun";
        runner.recycle(s);
        runner.cold(case)
    }

    /// The conditions on the 3D solver (see the module docs): the points in the order of `conditions`.
    pub(super) fn run_solver3(&self, conditions: &[Condition]) -> Result<Polar, Error> {
        self.check_solver3(conditions)?;
        // zero incidence first, then by Mach number and angle (stable); results go back to the order asked for
        let mut order: Vec<usize> = (0..conditions.len()).collect();
        order.sort_by(|&a, &b| {
            let (a, b) = (conditions[a], conditions[b]);
            (a.alpha_deg != 0.0).cmp(&(b.alpha_deg != 0.0)).then(a.mach.total_cmp(&b.mach)).then(a.alpha_deg.total_cmp(&b.alpha_deg))
        });
        let sorted: Vec<Condition> = order.iter().map(|&k| conditions[k]).collect();
        let machs: Vec<f64> = sorted.iter().map(|c| c.mach).collect();
        let flow = self.wall_flow(&machs)?;
        let level = self.resolution.level();
        let (mut points, backend, mut fallback) = self.solve3_on(self.backend, &sorted, level, flow, true)?;
        if self.estimate_uncertainty {
            // the neighbouring grid level, cold: the difference of the totals is the point's uncertainty (neither point failed)
            let cmp_level = if level >= 1 { level - 1 } else { level + 1 };
            let b = match backend {
                BackendUsed::Cpu => Backend::Cpu,
                BackendUsed::Gpu => self.backend,
            };
            let (other, _, note) = self.solve3_on(b, &sorted, cmp_level, flow, false)?;
            if let Some(note) = note {
                fallback = Some(format!("uncertainty sweep: {note}"));
            }
            for (p, o) in points.iter_mut().zip(&other) {
                if p.quality != Quality::Failed && o.quality != Quality::Failed {
                    p.cd_uncertainty = Some((p.drag.total - o.drag.total).abs());
                }
            }
        }
        let mut slots: Vec<Option<PolarPoint>> = (0..conditions.len()).map(|_| None).collect();
        for (&k, p) in order.iter().zip(points) {
            slots[k] = Some(p);
        }
        let points = slots.into_iter().map(|p| p.expect("every condition has a result")).collect();
        Ok(Polar { points, backend, fallback, motor_on: false })
    }

    /// `sorted` on `backend`: the points, the backend that ran them and, under [`Backend::Auto`], why the GPU was not used.
    fn solve3_on(
        &self,
        backend: Backend,
        sorted: &[Condition],
        level: u32,
        flow: Option<WallFlow>,
        keep: bool,
    ) -> Result<(Vec<PolarPoint>, BackendUsed, Option<String>), Error> {
        let cpu = || guard(|| self.sweep3(&Cpu3, sorted, level, flow, keep));
        match backend {
            Backend::Cpu => Ok((cpu()?, BackendUsed::Cpu, None)),
            Backend::Gpu => Ok((self.sweep3_gpu(sorted, level, flow, keep)?, BackendUsed::Gpu, None)),
            Backend::Auto => match self.sweep3_gpu(sorted, level, flow, keep) {
                Ok(p) => Ok((p, BackendUsed::Gpu, None)),
                Err(Error::Cancelled) => Err(Error::Cancelled),
                Err(e) => {
                    let why = match e {
                        Error::Internal(m) => m.replacen("solver panicked", "GPU solver panicked", 1),
                        e => format!("GPU unavailable: {e}"),
                    };
                    Ok((cpu()?, BackendUsed::Cpu, Some(why)))
                }
            },
        }
    }

    #[cfg(feature = "gpu")]
    fn sweep3_gpu(&self, sorted: &[Condition], level: u32, flow: Option<WallFlow>, keep: bool) -> Result<Vec<PolarPoint>, Error> {
        guard(|| self.sweep3(&Gpu3, sorted, level, flow, keep))
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep3_gpu(&self, _sorted: &[Condition], _level: u32, _flow: Option<WallFlow>, _keep: bool) -> Result<Vec<PolarPoint>, Error> {
        Err(Error::Unsupported("the `gpu` feature is not enabled".into()))
    }

    /// The sweep loop shared by the backends over `sorted`: the points of one domain and Mach number (the angles of attack) are a group, solved one after the other on one
    /// solver, the first from the free stream or its axisymmetric seed ([`Analysis::seed_axisymmetric`], [`Analysis::cold_point`]) and the later ones warm from the one before.
    /// A warm point that ends `Failed` or `Stalled` is re-run cold. With [`Analysis::warm_start`] the groups of a domain are solved in the order of
    /// [`warm_plan`] and the first point of a group starts from the converged field of its neighbour in the plan, carried to its Mach number ([`Carried`]); a group
    /// that has no neighbour (the first of a run of the plan, or the neighbour failed or did not settle) starts as it does without the flag. The points come back in the
    /// order of `sorted`.
    fn sweep3<R: Runner3>(&self, runner: &R, sorted: &[Condition], level: u32, flow: Option<WallFlow>, keep: bool) -> Result<Vec<PolarPoint>, Error> {
        use std::collections::HashMap;
        use std::time::Instant;
        // the groups: consecutive conditions of one domain and Mach number (the order of `sorted` keeps them together)
        let mut groups: Vec<(Kind, Vec<usize>)> = Vec::new();
        for (i, &c) in sorted.iter().enumerate() {
            let kind = self.kind_of(c);
            match groups.last_mut() {
                Some((k, members)) if *k == kind && sorted[members[0]].mach == c.mach => members.push(i),
                _ => groups.push((kind, vec![i])),
            }
        }
        // the steps in the order they are solved in: (group, the group whose field it starts from)
        let tune = WarmTuning::current();
        let mut steps: Vec<(usize, Option<usize>)> = Vec::with_capacity(groups.len());
        let mut from = 0;
        while from < groups.len() {
            let to = (from..groups.len()).find(|&g| groups[g].0 != groups[from].0).unwrap_or(groups.len());
            if self.warm_start {
                let machs: Vec<f64> = (from..to).map(|g| sorted[groups[g].1[0]].mach).collect();
                steps.extend(
                    warm_plan(&machs, tune.split, tune.sub_up, tune.max_ratio, tune.max_mach).into_iter().map(|s| (from + s.point, s.parent.map(|p| from + p))),
                );
            } else {
                steps.extend((from..to).map(|g| (g, None)));
            }
            from = to;
        }
        // the fields still wanted by a group that is not solved yet, and how many groups want each
        let mut wanted: Vec<usize> = vec![0; groups.len()];
        for &(_, parent) in &steps {
            if let Some(p) = parent {
                wanted[p] += 1;
            }
        }
        let mut carried: HashMap<usize, Carried> = HashMap::new();
        let mut points: Vec<Option<PolarPoint>> = (0..sorted.len()).map(|_| None).collect();
        // the axisymmetric seeds of the sweep by Mach number and tier
        let mut seeds = HashMap::new();
        for (g, parent) in steps {
            let (kind, members) = (groups[g].0, &groups[g].1);
            let mut state: Option<R::State> = None;
            let mut last = sorted[members[0]];
            // the `cfl_max` the last run of the group ended with
            let mut cap: Option<f64> = None;
            for (j, &i) in members.iter().enumerate() {
                let c = sorted[i];
                last = c;
                let t_case = Instant::now();
                let case = self.solver3_case(c, level, Self::tier3(c, self.resolution, flow), keep);
                let mut tm = FacadeTiming {
                    what: "point",
                    mach: c.mach,
                    alpha_deg: c.alpha_deg,
                    kind: match kind {
                        Kind::Wedge => "wedge",
                        Kind::Sector => "sector",
                        Kind::Half => "half",
                    },
                    case_s: t_case.elapsed().as_secs_f64(),
                    ..Default::default()
                };
                if let Some(s) = &mut state {
                    let t = Instant::now();
                    let r = runner.warm(s, &case, if self.warm_start && tune.keep_cfl { cap } else { None })?;
                    cap = Some(r.final_cfl_cap().unwrap_or(case.numerics.implicit.cfl_max));
                    tm.run_s = t.elapsed().as_secs_f64();
                    if r.stop_reason == StopReason::Cancelled {
                        return Err(Error::Cancelled);
                    }
                    if matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
                        points[i] = Some(self.point3(runner, s, &case, c, &r, keep, &mut tm)?);
                        record_facade(tm);
                        continue;
                    }
                }
                let t = Instant::now();
                let neighbour = parent.filter(|_| j == 0).and_then(|p| carried.get(&p)).map(|f| (f, &tune));
                let (r, s) = self.cold_point(runner, &case, c, kind, level, &mut seeds, neighbour, &mut tm)?;
                cap = Some(r.final_cfl_cap().unwrap_or(case.numerics.implicit.cfl_max));
                let mut next_warm = true;
                tm.run_s += t.elapsed().as_secs_f64();
                if r.stop_reason == StopReason::Cancelled {
                    return Err(Error::Cancelled);
                }
                if lift_quality(&r) == Quality::Failed {
                    next_warm = false;
                }
                tm.reused = runner.reused(&s);
                points[i] = Some(self.point3(runner, &s, &case, c, &r, keep, &mut tm)?);
                record_facade(tm);
                if j == 0 && wanted[g] > 0 && matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
                    if let Some(field) = runner.carry(&s, &case, cap.unwrap_or(case.numerics.implicit.cfl_max))? {
                        carried.insert(g, field);
                    }
                }
                drop_timed(runner, state.take(), c);
                state = next_warm.then_some(s);
            }
            drop_timed(runner, state, last);
            if let Some(p) = parent {
                wanted[p] -= 1;
                if wanted[p] == 0 {
                    carried.remove(&p);
                }
            }
        }
        Ok(points.into_iter().map(|p| p.expect("every condition has a point")).collect())
    }

    /// The point of the run `r` of `c` on `case`, left in `state`: [`Drag`] and [`PointDetail`] from the class forces and the wall samples, [`Forces`] at
    /// an angle of attack. What `drag` holds at an angle of attack is the axial force split by surface (its total is `forces.ca`, not a drag).
    #[allow(clippy::too_many_arguments)]
    fn point3<R: Runner3>(
        &self,
        runner: &R,
        state: &R::State,
        case: &Case3,
        c: Condition,
        r: &PointResult3,
        keep: bool,
        tm: &mut FacadeTiming,
    ) -> Result<PolarPoint, Error> {
        let (g, q) = (runner.grid(state), self.area_ratio());
        let t_post = std::time::Instant::now();
        let post = runner.post(state, case)?;
        tm.post_s = t_post.elapsed().as_secs_f64();
        let t_host = std::time::Instant::now();
        let p_inf = case.freestream()[4];
        let q_inf = 0.5 * c.mach * c.mach;
        let d = &post.drag;

        // the guards: the state, the outer boundaries, the base
        let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
        for (cell, u) in post.u.iter().enumerate() {
            if g.part[cell] == PART_FLUID {
                let w = cons_to_prim3(u);
                min_rho = min_rho.min(w[0]);
                min_p = min_p.min(w[4]);
            }
        }
        let farfield_dp = farfield_dp(g, &post.u, p_inf);
        let samples = wall_samples(g, &post.u);
        let cp = |pw: f64| (pw - p_inf) / q_inf;
        let (mut base_area, mut base_cp) = (0.0, 0.0);
        for w in samples.iter().filter(|w| w.class == Wall::Base) {
            base_area += w.face.area;
            base_cp += cp(w.pw) * w.face.area;
        }
        let cp_base = if base_area > 0.0 { base_cp / base_area } else { f64::NAN };
        let has_base = case.grid.base.is_some();

        // pressure and friction by surface; the friction is the RANS run's shear (zero, and `None`, for Euler)
        let viscous = case.viscous.is_some();
        let fr = &d.friction;
        let forebody = (d.nose + d.aft.iter().sum::<f64>()) * q;
        let fin_pressure = (d.fin_sides + d.fin_edges + d.fin_tips) * q;
        let fin_friction = d.fin_friction() * q;
        let body_friction = (d.body_friction() + fr.base) * q;
        let fins = self.fins.is_some().then_some(FinDrag { friction: fin_friction, pressure: fin_pressure });
        let base = d.base * q;
        let total = forebody + body_friction + base + fin_pressure + fin_friction;
        let drag = Drag { forebody_pressure: forebody, friction: viscous.then_some(body_friction), base, base_modelled: false, fins, total };

        let forces = (c.alpha_deg != 0.0).then(|| {
            let (cn, ca, x_cp) = (r.cn * q, r.ca * q, r.x_cp * self.d_nose);
            Forces { cn, cm: if x_cp.is_finite() { -cn * x_cp / self.d_ref } else { 0.0 }, x_cp, ca }
        });
        let finite = total.is_finite() && forces.is_none_or(|f| f.cn.is_finite() && f.ca.is_finite());
        let quality = quality3(r.stop_reason, c.mach, min_rho, min_p, finite, farfield_dp, cp_base);

        let mut warnings = Vec::new();
        if (0.7..1.05).contains(&c.mach) && self.has_sharp_corner() {
            warnings.push(Warning::SharpCornerTransonic);
        }
        if farfield_dp > 0.02 {
            warnings.push(Warning::FarFieldDisturbed { dp: farfield_dp });
        }
        // a run the CFL controller took back is a point obtained on a second attempt at other numerics, and says so
        if let (Some(first), Some(last)) = (r.cfl_events.first(), r.cfl_events.last()) {
            warnings.push(Warning::RestartedAtLowerCfl { rollbacks: r.cfl_events.len(), cfl_from: first.cfl_before, cfl_to: last.cfl_after });
        }

        // the wall Cp along the body of an axisymmetric or sector domain (a half domain has a ring of faces at every station: none)
        let x_base = self.body.x_base();
        let mut wall: Vec<WallCp> = if c.alpha_deg == 0.0 {
            samples
                .iter()
                .filter(|w| matches!(w.class, Wall::Body(_)) && w.face.c[0] < x_base)
                .map(|w| WallCp { x: w.face.c[0] * self.d_nose, r: w.face.c[1].hypot(w.face.c[2]) * self.d_nose, cp: cp(w.pw) })
                .collect()
        } else {
            Vec::new()
        };
        wall.sort_by(|a, b| a.x.total_cmp(&b.x));
        let detail = PointDetail {
            iterations: r.iterations,
            residual_drop_orders: r.drop_orders,
            stop_reason: r.stop_reason,
            cd_nose: d.nose * q,
            cd_aft: d.aft.iter().map(|a| a * q).collect(),
            cd_friction: if viscous { body_friction } else { 0.0 },
            cd_base_solver: if has_base { base } else { f64::NAN },
            cp_base: if has_base { cp_base } else { f64::NAN },
            wall,
            y_plus_max: if viscous && r.y_plus_max.is_finite() { r.y_plus_max } else { f64::NAN },
            farfield_dp,
            cfl_limit: Some(r.final_cfl_cap().unwrap_or(case.numerics.implicit.cfl_max)),
            rollbacks: r
                .cfl_events
                .iter()
                .map(|e| Rollback { iteration: e.iteration, restored_to: e.restored_to, cfl_before: e.cfl_before, cfl_after: e.cfl_after })
                .collect(),
        };
        tm.host_s = t_host.elapsed().as_secs_f64();
        let t_field = std::time::Instant::now();
        let field3 = (keep && self.keep_fields).then(|| {
            let mut f = field3_of(case, g, &post.u, r.iterations, self.field_scale());
            if !post.eddy.is_empty() {
                f.eddy_ratio = post.eddy.clone();
                for (w, cf) in f.walls.iter_mut().zip(&post.cf) {
                    w.cf = *cf;
                }
            }
            f
        });
        tm.field_s = t_field.elapsed().as_secs_f64();
        Ok(PolarPoint { mach: c.mach, alpha_deg: c.alpha_deg, drag, forces, detail, quality, warnings, cd_uncertainty: None, field: None, field3 })
    }
}

/// Largest relative deviation of the pressure from the free stream over the fluid cells of the outer boundaries (inflow, outflow, far field): `|p / p_inf - 1|`.
fn farfield_dp(g: &Grid3, u: &[Cons3], p_inf: f64) -> f64 {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let mut dp: f64 = 0.0;
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                if i != 0 && i != ni - 1 && j != nj - 1 {
                    continue;
                }
                let cell = (k * nj + j) * ni + i;
                if g.part[cell] == PART_FLUID {
                    dp = dp.max((cons_to_prim3(&u[cell])[4] / p_inf - 1.0).abs());
                }
            }
        }
    }
    dp
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{AftSpec, NoseShape, NoseSpec};

    #[test]
    fn the_facades_area_factor_is_the_engines_nose_to_largest_area_for_the_default_reference() {
        // a nose of 0.1 m and a tube of 0.14 m: the largest section is the tube's
        let spec = BodySpec {
            nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 },
            aft: vec![AftSpec::Transition { length: 0.1, aft_diameter: 0.14 }, AftSpec::Tube { length: 0.5 }],
        };
        let a = Analysis::new(&spec).unwrap();
        assert!((a.area_ratio() - a.body.nose_to_largest_area()).abs() < 1e-12, "{} {}", a.area_ratio(), a.body.nose_to_largest_area());
        // an explicit reference diameter is the facade's own business
        let b = Analysis::new(&spec).unwrap().reference_diameter(0.1);
        assert!((b.area_ratio() - 1.0).abs() < 1e-12);
    }

    /// The ANF-like rocket on the tiny sting grid (20 k cells), Euler, CPU: a sector at zero incidence and a half domain at 4 degrees.
    fn tiny_finned() -> Analysis {
        const D: f64 = 0.04;
        let nose_len = 0.5 * D / 10f64.to_radians().tan();
        let spec =
            BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] };
        let fins = crate::spec::FinSpec {
            count: 4,
            root_chord: D,
            tip_chord: D,
            span: D,
            sweep: 0.0,
            thickness: 0.08 * D,
            roughness: 0.0,
            root_te_to_base: 0.0,
            profile: crate::spec::FinProfile::Square,
        };
        Analysis::new(&spec)
            .unwrap()
            .fins(fins)
            .engine(Engine::Solver3)
            .backend(Backend::Cpu)
            .wake(Wake::Sting)
            .resolution(Resolution::Coarse)
            .case3_hook(|c| {
                let s = &mut c.grid.sting;
                s.n_up = 10;
                s.n_nose = 16;
                s.n_seg = vec![12];
                s.n_ext = 10;
                s.nj = 24;
                c.grid.nk = 8;
            })
            .max_iterations(3000)
    }

    /// Only the finned sector at zero incidence is seeded, unless `seed_axisymmetric(false)`: not the half domain at incidence (slower than its cold run), not a finless body,
    /// not the wedge; and a half-domain point is the cold run whatever the flag says.
    #[test]
    fn only_the_finned_sector_is_seeded_and_the_opt_out_is_cold() {
        let on = tiny_finned();
        let off = tiny_finned().seed_axisymmetric(false);
        assert!(on.seed_axisymmetric && !off.seed_axisymmetric);
        assert_eq!([Kind::Wedge, Kind::Sector, Kind::Half].map(|k| on.seeded(k)), [false, true, false]);
        assert_eq!([Kind::Wedge, Kind::Sector, Kind::Half].map(|k| off.seeded(k)), [false, false, false]);
        let mut finless = tiny_finned();
        finless.fins = None;
        assert_eq!([Kind::Wedge, Kind::Sector, Kind::Half].map(|k| finless.seeded(k)), [false, false, false]);

        let conditions = [Condition::new(2.0, 4.0)];
        let _ = take_facade_timings();
        let a = on.run(&conditions).unwrap();
        let b = off.run(&conditions).unwrap();
        let timings = take_facade_timings();
        assert!(timings.iter().all(|t| t.seed.is_empty() && t.seed_iters == 0), "{timings:?}");
        assert_eq!(format!("{:?}", a.points[0].forces), format!("{:?}", b.points[0].forces));
        assert_eq!(a.points[0].detail.iterations, b.points[0].detail.iterations);
    }

    /// The same on the sector at zero incidence (the Euler tiny ANF sector, CPU): its wedge is the body's on the same meridian, its coefficient the axial force, and a
    /// seeded run carries on from the wedge's field without the first-order start-up.
    #[test]
    fn a_seeded_sector_reaches_the_axial_force_of_the_cold_run() {
        let mut sector = Case3::tiny_anf(2.0, 0.0);
        sector.grid.domain = crate::engine::grid3::Domain3::Sector;
        let mut wedge = sector.clone();
        wedge.grid.fins = None;
        let wedge = wedge.on_wedge();
        let seed = Cpu3.seed(&wedge).unwrap().expect("the wedge converges");
        let (cold, _) = Cpu3.cold(&sector).unwrap();
        let (seeded, _) = Cpu3.cold_from(&sector, &Start::Seed(&seed)).unwrap();
        println!(
            "wedge {} iterations; sector cold {} ({:?}) CA {:.5}, seeded {} ({:?}) CA {:.5}",
            seed.iterations, cold.iterations, cold.stop_reason, cold.ca, seeded.iterations, seeded.stop_reason, seeded.ca
        );
        assert!(matches!(lift_quality(&seeded), Quality::Converged | Quality::Stationary));
        assert!((seeded.ca / cold.ca - 1.0).abs() < 0.01, "CA {} vs {}", seeded.ca, cold.ca);
    }

    /// A sector started from the converged field of a neighbouring Mach number (`Start::Neighbour`, either rescale) reaches the axial force of its own cold run, and a carried
    /// field only fits a case on the same grid and incidence.
    #[test]
    fn a_sector_started_from_a_neighbouring_mach_number_reaches_the_axial_force_of_the_cold_run() {
        let sector = |mach: f64| {
            let mut c = Case3::tiny_anf(mach, 0.0);
            c.grid.domain = crate::engine::grid3::Domain3::Sector;
            c
        };
        let (near, far) = (sector(2.0), sector(1.6));
        let (r_near, s_near) = Cpu3.cold(&near).unwrap();
        let carried = Cpu3.carry(&s_near, &near, near.numerics.implicit.cfl_max).unwrap().expect("a sound field");
        assert!(carried.fits(&far) && !carried.fits(&Case3::tiny_anf(1.6, 0.1)));
        let (cold, _) = Cpu3.cold(&far).unwrap();
        for mode in [Rescale::Perturbation, Rescale::Velocity] {
            let (warm, _) = Cpu3.cold_from(&far, &Start::Neighbour { from: &carried, rescale: mode, cfl: None }).unwrap();
            println!(
                "M 2.0 cold {} its CA {:.5}; M 1.6 cold {} its CA {:.5}, from M 2.0 ({mode:?}) {} its ({:?}) CA {:.5}",
                r_near.iterations, r_near.ca, cold.iterations, cold.ca, warm.iterations, warm.stop_reason, warm.ca
            );
            assert!(matches!(lift_quality(&warm), Quality::Converged | Quality::Stationary), "{:?}", warm.stop_reason);
            assert!((warm.ca / cold.ca - 1.0).abs() < 0.01, "CA {} vs {}", warm.ca, cold.ca);
        }
    }
}
