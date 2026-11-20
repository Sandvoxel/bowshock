//! A single (Mach number, incidence) 3D Euler case: sting-mounted body with optional thick fins, grid parameters and numerics, and the
//! CPU solver built from them. Lengths are in nose diameters (`d = 1`).

use crate::engine::body_grid::{BaseBody, Body, Nose};
use crate::engine::case::Case;
use crate::engine::field3::Snapshots3;
use crate::engine::grid3::{body_grid3, Domain3, FinStack, Fins3, Grid3Params, WEDGE_ANGLE};
use crate::engine::meshing::{base_grid, draft_coarsen, motor_rows, resolve_walls, sting_grid, wall_spacing, Jet};
use crate::error::Error;
use crate::progress::ProgressSink;
use crate::spec::FinProfile;
use bowshock_cfd::gas3::freestream3;
use bowshock_cfd::grid3::{Grid3, PartRoughness};
use bowshock_cfd::implicit_params::{DtMode, ImplicitParams, LineSolve3};
use bowshock_cfd::limiter::Limiter;
use bowshock_cfd::solver::{Bc, Scheme};
use bowshock_cfd::solver3::{Config3, RansOpts3, Scheme3, Solver3};
use bowshock_cfd::sst::{Turbulence, Viscous, WallThermal};
use bowshock_cfd::transition3::Transition;

/// `cfl_max` of a wall-resolved (viscous) **3D** grid, a half domain or a sector (the wedge, one azimuthal cell, keeps `Numerics3::viscous`'s 50 below Mach 1 and 20 above): 10.
/// The three-factor implicit step has a CFL limit on every 3D grid tried, finned or not, between 30 and 40 at Mach 0.5 and lower supersonic (CPU f64, Draft tier of the example rocket: the
/// sector holds at 10, 15, 20 and 30 and is non-finite at iteration 200 at 40 and 50, as the finless uniform-azimuth half domain is at 50; the fin section, sweep, thickness, the Euler
/// equations, laminar flow, `first_order_iters` 400, `sub_iters` 6 and the implicit entropy fix do not change it; only the scalar (Rusanov) implicit dissipation holds at 50, and converges
/// slower). The defaults of 50 / 20 went non-finite after the ramp passed 40 (iteration 190 to 215 at Mach 0.1 to 0.9 on the GPU, f32, Coarse and Draft sectors) and, at 20, at iteration 411 /
/// 932 / 4508 (Mach 3 Draft, Mach 3 Coarse, Mach 2.3 Coarse). GPU f32, Draft sector, the ten Mach numbers 0.1 to 3.0 at 10: every one finite; 8 stop on the mean rule after 6890 to 11830
/// iterations, Mach 0.1 and 0.8 (unsteady base wake) reach the 12000 cap; at 20 Mach 3 is non-finite at 411 and the others stop after 25 to 35 % fewer iterations, which a cap
/// of 20 would buy where it holds. The Lift-tier gates (G-L1, G-F5) ran at 10.
pub const VISCOUS_3D_CFL_MAX: f64 = 10.0;

/// `cfl_max` of a viscous 3D domain (half domain, sector) with the DDADI line solve ([`Case3::with_tier`]): the rollback controller starts here.
pub const DDADI_CFL_MAX: f64 = 100.0;

/// [`bowshock_cfd::implicit_params::ImplicitParams::lhs_dis_boost`] of the DDADI line solve on the viscous 3D domains (CPU f64 viscous block: 0.7 is the smallest boost that holds
/// CFL 100-300 without the period-2 cycle at Mach 2; at Mach 0.5 with the low-Mach preconditioning any value from 0 holds).
pub const DDADI_DIS_BOOST: f64 = 0.7;

/// Numerical settings and stop-rule tolerances of a 3D run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Numerics3 {
    pub limiter: Limiter,
    pub low_mach: bool,
    pub implicit: ImplicitParams,
    /// First-order start-up iterations.
    pub first_order_iters: usize,
    pub max_iter: usize,
    /// Stationary rule: relative CN range over the last `window` iterations.
    pub rel_tol: f64,
    /// Window \[iterations\] of the stationary rule (a multiple of [`crate::engine::run3::SAMPLE_EVERY`]).
    pub window: usize,
    /// Iterations (from the start of a cold run) before the stationary rule may fire (the rule also needs one full window).
    pub stationary_min_iters: usize,
    /// The same floor for a warm-started run, counted from the warm start.
    pub warm_min_iters: usize,
    /// Mean-stationary rule ([`crate::engine::run3::StopRule3`]): length of one averaging window \[iterations\] (`0`: off). A run stops [`crate::engine::result::StopReason::MeanStationary`]
    /// when the means of CA (at incidence instead: of CN and of the moment about the nose, within `incidence_tol`) over two consecutive windows agree within `mean_tol` (relative, plus 1e-4 absolute) and
    /// both windows lie after iteration `mean_from`; the result is then the mean of the second window. For the oscillating subsonic base wake, whose period (50 to 220 iterations)
    /// the range rule cannot see.
    pub mean_window: usize,
    pub mean_from: usize,
    pub mean_tol: f64,
    /// The mean rule at incidence (a half domain at an angle of attack) uses these instead of `mean_window`, `mean_from` and `mean_tol`, and judges CN and the moment about the nose
    /// only (CA is not a product of an angle-of-attack point). Measured on the example rocket's half domain at 2 degrees (Lift tier, Coarse, Mach 0.5, 1.1, 2.3): windows of
    /// 1000 from 1000 stop at 3000 iterations instead of 6000, CN within 0.01 % and `x_cp` within 0.003 cal of the 5e-4 reference. `incidence_window` 0: off.
    pub incidence_window: usize,
    pub incidence_from: usize,
    pub incidence_tol: f64,
    /// Explicit 3D line solve overriding the backend default (`None`: [`Numerics3::implicit_for`]).
    pub line_solve: Option<LineSolve3>,
    /// Static-pressure outflow `p = p_inf` at the last `i` plane instead of the characteristic ghost (the plates of the viscous gates; the body cases of the 2D
    /// engine, and so these, use the characteristic outflow: `false`). Read by the viscous solver only.
    pub pressure_outflow: bool,
    /// The CFL the windows and floors above are calibrated for (10: the Euler numerics' cap; 20: the viscous ones). The settling of a run is pseudo-time, it runs at the CFL (the
    /// iterations to a given error times `cfl_max` are constant on the G-F1 sector at 20, 30 and 50), so a drift-based rule that watches a window of iterations at a lower CFL sees
    /// less drift for the same remaining error and fires early (the application's rule at `cfl_max` 10 on that sector: 1.5 % high, at 20: 0.9 %). [`crate::engine::run3::StopRule3`]
    /// multiplies every window and floor by `max(1, cfl_ref / CFL in force)`: unchanged at and above `cfl_ref`, twice as long at 10 for a viscous run. `0`: off.
    pub cfl_ref: f64,
    /// The runners answer a failed stretch (a residual that is not finite or above 1e5, a non-finite force monitor) by going back to the last snapshot with `cfl_max` x 0.7, down
    /// to `run3::CFL_FLOOR` (`run3::CflControl`): the default, with the engine's `cfl_max`. Off: no rollback, and [`Case3::without_cfl_control`] takes the fixed [`VISCOUS_3D_CFL_MAX`].
    pub cfl_control: bool,
    /// Extrapolated-drift rule ([`crate::engine::run3::StopRule3`], [`crate::engine::result::StopReason::Settled`]): the judged coefficients (CA; at incidence CN and the moment about the
    /// nose) are averaged over blocks of `drift_block` summed CFL (sum over the iterations of the CFL in force: the settling of a run is pseudo-time, so a block is the same stretch of it at
    /// any CFL); three consecutive block means `m1, m2, m3` give the ratio `rho = (m3 - m2) / (m2 - m1)` of a geometric tail and the remaining change `e = (m3 - m2) rho / (1 - rho)`.
    /// The run stops when, for every judged coefficient, `0 < rho < 0.95`, `rho` is within 0.15 of the previous block's and `|e| <= drift_tol |m3| + 1e-4`. Replayed offline on the
    /// stored G-F1 sector histories (Mach 0.5 and 2, CFL 20-50, 8 000-12 000 iterations; blocks of 500 iterations at CFL 20, `drift_tol` 5e-3): it stops after 2 000-6 000 iterations
    /// (mean 4 800, against 6 000-8 000 for the mean rule), 0.2-1.1 % from the converged CA. `0`: off.
    pub drift_block: f64,
    pub drift_tol: f64,
}

impl Numerics3 {
    /// Van Albada with the low-Mach correction below Mach 0.85, minmod above; the implicit defaults with `cfl_max` 10 for every Mach
    /// number (the three-factor scheme diverged on the coarse ANF grid after the ramp passed about 15, at 20 and 30; 10 and 5 converge;
    /// measured again with the frozen far-field Jacobian: `cfl_max` 15 saves 6 to 9 % of the iterations, 20 stalls a point of the example
    /// table and moves CN by 1 %). The stationary window is 250 iterations with a floor of one window: CN of the ANF and the example rocket
    /// settles to 1e-4 within 200 to 300 iterations, and against the 500-iteration window with a 1000-iteration floor the table moved by
    /// at most 0.18 % in CN and 0.003 cal in `x_cp` while using 40 % fewer iterations (a window of 200 moved M 0.95 by 0.54 %).
    ///
    /// The line solve is chosen in one place, [`Numerics3::implicit_for`]: block lines on the CPU, the diagonalised form on the GPU.
    pub fn auto(mach: f64, level: u32) -> Numerics3 {
        let subsonic = mach < 0.85;
        Numerics3 {
            limiter: if subsonic { Limiter::VanAlbada } else { Limiter::Minmod },
            low_mach: subsonic,
            implicit: ImplicitParams { cfl_max: 10.0, ..ImplicitParams::DEFAULT },
            first_order_iters: 100,
            max_iter: 8000 << level,
            rel_tol: 0.002,
            window: 250,
            stationary_min_iters: 250,
            warm_min_iters: 250,
            mean_window: 1500,
            mean_from: 1000,
            mean_tol: 1e-2,
            incidence_window: 1000,
            incidence_from: 1000,
            incidence_tol: 2e-3,
            line_solve: None,
            pressure_outflow: false,
            cfl_ref: 10.0,
            cfl_control: true,
            drift_block: 0.0,
            drift_tol: 5e-3,
        }
    }

    /// The settings of an Euler wedge, the 2D engine's `Numerics::auto` for the base-wake grid: below Mach 0.85 the subsonic ones ([`Numerics3::auto`] with the implicit
    /// defaults' CFL limit 30 and `10 000 << level` iterations), from Mach 0.85 the transonic ones (60 000 iterations; 300 first-order iterations below Mach 1.2 and the 100 of
    /// [`Numerics3::auto`] above: 300 first-order iterations at Mach 2 take the Euler wake of the boattailed Arcas to a second steady state, base `CD` 0.098 against the 2D
    /// reference's 0.059 (G-W1 `euler-base-arcas-M2.00`, found by bisection to the viscous wiring commit 5f89ed1, which took the 2D numerics over); the 2D explicit
    /// scheme of the transonic regime has no 3D counterpart, the implicit CFL limit stays 10). Block lines. What [`Case3::wedge`] runs with; the facade's Euler tables of the
    /// frozen 2D reference (G-V4 table-euler, G-W1) are the 2D engine's numbers.
    pub fn euler_wedge(mach: f64, level: u32) -> Numerics3 {
        let mut n = Numerics3::auto(mach, level);
        if mach < 0.85 {
            n.implicit = ImplicitParams::DEFAULT;
            n.max_iter = 10_000 << level;
        } else {
            n.first_order_iters = if mach < 1.2 { 300 } else { n.first_order_iters };
            n.max_iter = 60_000;
        }
        n.line_solve = Some(LineSolve3::Block);
        n
    }

    /// The settings of a wall-resolved RANS run, as the 2D engine's `Numerics::viscous` has them: van Albada below Mach 0.85 and minmod above, the low-Mach
    /// correction and (below Mach 1) the low-Mach preconditioning of the implicit left-hand side, `cfl_max` 50 (20 from Mach 1: the supersonic wake diverges at 50),
    /// `DtMode::Sum`, three factored solves per iteration (`sub_iters` 3). One is enough on the wedge ([`Numerics3::from_2d`]) but not on a finned grid: the small finned sector
    /// (short body, 4 fins, 324k cells, Mach 0.5, `cfl_max` 50) with `sub_iters` 1 went non-finite at iteration 370, 170 after the CFL ramp reached 50 (the residual jumped from 80
    /// to 260 at 300), and with 3 its residual holds at 80 over iterations 200 to 300; at Mach 2 (`cfl_max` 20) 1 and 3 agree to 450 iterations. 200 first-order iterations,
    /// the **block** lines (the diagonal solve is unmeasured on viscous wedges and stalls with the preconditioning on plates), the characteristic outflow. The stationary rule
    /// waits 1000 iterations and a window of 1000 (friction settles slowly).
    pub fn viscous(mach: f64, level: u32) -> Numerics3 {
        let subsonic = mach < 0.85;
        Numerics3 {
            limiter: if subsonic { Limiter::VanAlbada } else { Limiter::Minmod },
            low_mach: subsonic,
            implicit: ImplicitParams {
                dt_mode: DtMode::Sum,
                cfl_max: if mach < 1.0 { 50.0 } else { 20.0 },
                sub_iters: 3,
                low_mach_precond: mach < 1.0,
                ..ImplicitParams::DEFAULT
            },
            first_order_iters: 200,
            max_iter: 80_000 << level,
            rel_tol: 5e-4,
            window: 1000,
            stationary_min_iters: 1000,
            warm_min_iters: 1000,
            mean_window: 2000,
            mean_from: 2000,
            mean_tol: 5e-3,
            incidence_window: 1000,
            incidence_from: 1000,
            incidence_tol: 2e-3,
            line_solve: Some(LineSolve3::Block),
            cfl_ref: 20.0,
            cfl_control: true,
            pressure_outflow: false,
            drift_block: 10_000.0,
            drift_tol: 5e-3,
        }
    }

    /// The numerics of a 2D engine case on the wedge: its limiter, low-Mach flags, first-order iterations, implicit parameters (with the block lines) and outflow; **one factored
    /// solve** per iteration for a viscous case **below Mach 1**: on the CPU wedge (RANS-table body at Mach 0.1, 0.9, 2.3 and G3 at Mach 0.5, 1.5, level 0) 1, 2 and 3 need the same
    /// iterations to within 5 % (2050 / 2900 / 2850 / 3900 / 3400 for 1) and reach the same `C_D` to 0.03 %, at 0.66 to 0.71 of the time per iteration of 3; at Mach 0.1 the converged
    /// `C_D` of 2 and 3 is 1.5 % above the 2D solver's and that of 1 is 0.3 % above it (the residual of 2 and 3 stalls 14 times higher). **Not from Mach 1**: at the medium level
    /// (level 1) `sub_iters` 1 is non-finite on the Arcas fins-off body at Mach 2.3 (iteration 250, density 1e71) and on TP-2005 at Mach 1.65 (pb 1e104), where 3 passes
    /// (`solver3d`'s G-V4 and G-J on the merged build); the level-0 supersonic points above did not show it, so the supersonic wedge keeps the 2D case's 3. The stop rule is
    /// [`Numerics3::auto`]'s for an Euler case and [`Numerics3::viscous`]'s for a viscous one, except that
    /// a case with a `drag_rel_tol` (the facade's `drag_tolerance`) is stopped as the 2D engine stops it: the relative range of the monitor below that tolerance over
    /// [`DRAG_REL_WINDOW`] iterations. On one CPU wedge history per point (RANS-table body at Mach 0.1, 0.9, 2.3, G3 at Mach 0.5, 1.5) the rule of 5e-4 over 1000 iterations
    /// fires at 2050 / 3900 / 3400 / 2900 / 2850 iterations and the 2D rule at 1350 / 1850 / 1650 / 2050 / 1800 (the fixture's 2D runs: 1200 / 1850 / 1650), at a `C_D` that is
    /// 0.0 % to 0.24 % apart.
    pub fn from_2d(case: &Case, level: u32) -> Numerics3 {
        let mut n = if case.viscous.is_some() { Numerics3::viscous(case.mach, level) } else { Numerics3::auto(case.mach, level) };
        n.limiter = case.numerics.limiter;
        n.low_mach = case.numerics.low_mach;
        // not the 300 of the 2D transonic regime for an Euler case from Mach 1.2: [`Numerics3::euler_wedge`] explains why
        if case.viscous.is_some() || case.mach < 1.2 {
            n.first_order_iters = case.numerics.first_order_iters;
        }
        n.max_iter = case.numerics.max_iter;
        if let Scheme::Implicit(p) = case.numerics.scheme {
            let sub_iters = if case.viscous.is_some() && case.mach < 1.0 { 1 } else { p.sub_iters };
            n.implicit = ImplicitParams { line_solve: LineSolve3::Block, sub_iters, ..p };
        }
        if let (Some(tol), true) = (case.numerics.drag_rel_tol, case.viscous.is_some()) {
            (n.rel_tol, n.window, n.stationary_min_iters, n.warm_min_iters) = (tol, DRAG_REL_WINDOW, DRAG_REL_WINDOW, DRAG_REL_WINDOW);
        }
        n.line_solve = Some(LineSolve3::Block);
        n.pressure_outflow = case.numerics.outflow == Bc::PressureOutflow;
        n
    }

    /// The implicit parameters for a backend with the line solve resolved: `line_solve` if set, else the backend default.
    ///
    /// The default on the GPU is the diagonalised form with one defect-correction pass against the block operator
    /// ([`LineSolve3::DiagonalCorrected`]; `sub_iters` is then not used). It was measured on the GPU only: on the ANF case (level 0, f32, RTX 3090) a step costs 1935 us instead of
    /// 3151 us, the iterations and CN equal those of the block scheme (CN within 0.01 %) at Mach 0.5, 0.8, 2 and 3, 1.56x faster to the stop rule.
    /// The f64 CPU solver with it diverges on the tiny ANF grid at `cfl_max` 10 (it converges at 5), so the CPU default is the block solve. A wedge is always
    /// `Block` ([`Case3::on_wedge`] sets `line_solve`): the diagonal solve does not carry the wedge's pressure-source block.
    pub fn implicit_for(&self, gpu: bool) -> ImplicitParams {
        let default = if gpu { LineSolve3::DiagonalCorrected } else { LineSolve3::Block };
        ImplicitParams { line_solve: self.line_solve.unwrap_or(default), ..self.implicit }
    }
}

/// Free-stream and surface data a wall-resolved 3D grid is sized from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallFlow {
    /// `rho c d / mu` of the free stream (`d` the nose diameter).
    pub re_c: f64,
    /// Largest Mach number of the table the grid serves (the wall spacing is for the highest Reynolds number).
    pub mach_max: f64,
    /// Equivalent sand-grain roughness heights of the body and of the fins \[d\].
    pub ks_body: f64,
    pub ks_fin: f64,
    /// Sutherland's constant over the free-stream temperature, `110.4 K / T_inf` ([`SUTHERLAND_SEA_LEVEL`] for the ISA at sea level): the viscosity law of the RANS run.
    pub sutherland: f64,
}

/// `110.4 K / 288.15 K`: the Sutherland ratio of the ISA at sea level.
pub const SUTHERLAND_SEA_LEVEL: f64 = 110.4 / 288.15;

/// Cells over the fin root chord of the Draft tier at level 0 (Coarse and Fine: [`COARSE_CHORD_CELLS`]). 20 left the fin pressure 8-10 % low (example rocket, Mach 1.0 / 1.7: 0.1582 / 0.1534 against
/// 0.1709 / 0.1674 on Coarse); 32 closes it (0.1720 / 0.1683) for 11 % more cells, 17 % more wall and the same memory, and puts Draft within 0.1 % of Coarse at Mach 1.7 (GPU, cfl 20).
pub const DRAFT_CHORD_CELLS: usize = 32;

/// Cells over the fin root chord of the Coarse and Fine tiers at level 0 (Lift keeps the grid default, 32). 32 under-resolves the fin leading edge (2-7 % of CD0 against the
/// grid-converged value, DESIGN3D item 47); 64 lands within 0.8 %.
pub const COARSE_CHORD_CELLS: usize = 64;

/// Window \[iterations\] of the relative drag rule on a wedge ([`Case3::with_drag_tolerance`]; the number of the 2D engine's rule).
pub const DRAG_REL_WINDOW: usize = 500;

/// Target `y+` of the first cell of the Draft tier on the body and on the fin sides.
pub const DRAFT_Y_PLUS: f64 = 40.0;

/// What a 3D grid is resolved for: the wall spacing of the body and of the fin sides, and the azimuthal policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tier {
    /// Inviscid: no wall spacing, uniform azimuthal cells.
    Euler,
    /// `Resolution::Coarse` of a wall-resolved case: body `y+` ~ 1, fin sides `y+` ~ 15 (automatic wall treatment), the fin stack of [`FinStack`].
    Coarse(WallFlow),
    /// Body and fin sides `y+` ~ 1.
    Fine(WallFlow),
    /// A cheaper RANS solve: wall functions on the body and on the fins (first cell at the `y+` of the second field, [`DRAFT_Y_PLUS`] by default, through the automatic wall treatment, as the lift tier), a coarser axial grid
    /// ([`crate::engine::meshing::DRAFT_AXIAL`]) and a smaller domain ([`crate::engine::meshing::DRAFT_DOMAIN`]); with fins the fin stack at that `y+`.
    Draft(WallFlow, f64),
    /// The lift tier of a half domain: body and fins `y+` ~ 60 (wall functions); without fins uniform azimuthal cells, `32 << level` over the half domain, with fins the
    /// fin stack of [`FinStack`] at `y+` 60 (a uniform azimuth puts the first cell of a fin side at thousands of wall units).
    Lift(WallFlow),
}

impl Tier {
    /// The Draft tier of `flow` at [`DRAFT_Y_PLUS`].
    pub fn draft(flow: WallFlow) -> Tier {
        Tier::Draft(flow, DRAFT_Y_PLUS)
    }

    /// The flow the walls are sized for (`None` for [`Tier::Euler`]).
    pub fn flow(&self) -> Option<&WallFlow> {
        match self {
            Tier::Euler => None,
            Tier::Coarse(f) | Tier::Fine(f) | Tier::Lift(f) | Tier::Draft(f, _) => Some(f),
        }
    }

    /// Target `y+` of the first cell on the body and on the fin sides (`None` for [`Tier::Euler`]).
    pub fn y_plus(&self) -> Option<(f64, f64)> {
        match self {
            Tier::Euler => None,
            Tier::Coarse(_) => Some((1.0, 15.0)),
            Tier::Fine(_) => Some((1.0, 1.0)),
            Tier::Lift(_) => Some((60.0, 60.0)),
            Tier::Draft(_, y) => Some((*y, *y)),
        }
    }
}

/// One Mach number and incidence on one body.
#[derive(Clone, Debug)]
pub struct Case3 {
    pub mach: f64,
    /// Incidence \[rad\]; the air moves towards `+z`.
    pub alpha: f64,
    pub body: BaseBody,
    pub grid: Grid3Params,
    pub numerics: Numerics3,
    pub tier: Tier,
    /// Where the run reports its progress and can be stopped; `None`: silent.
    pub progress: Option<ProgressSink>,
    /// Where the run hands snapshots of its flow field; `None`: none.
    pub snapshots: Option<Snapshots3>,
    /// The motor jet at the nozzle exit of the base ([`Case3::with_motor`]); `None`: power off.
    pub jet: Option<Jet>,
    /// The viscous (RANS, SST) equations: set by [`Case3::with_tier`] for the wall-resolved tiers and by [`Case3::wedge_of`] for a viscous 2D case, `None`: Euler
    /// ([`Case3::euler_equations`] keeps a wall-resolved grid and drops the viscous terms). Only `re_c` and `sutherland` are read by the 3D solver.
    pub viscous: Option<Viscous>,
    /// Thermal condition of the no-slip walls of the viscous equations ([`WallThermal`]): adiabatic by default; [`Case3::with_wall_thermal`]. Not part of the grid: the wall
    /// spacing of the tiers is sized for the adiabatic wall, and a cold wall (higher wall density, lower wall viscosity) raises the `y+` of the first cells.
    pub wall_thermal: WallThermal,
    /// Prescribed transition of the viscous equations ([`Case3::with_transition`]): laminar walls upstream of the given stations, turbulent downstream; `None` (default): fully
    /// turbulent from the nose tip.
    pub transition: Option<Transition>,
}

/// What the solver needs of the motor jet at the nozzle exit faces (`Wall::Jet`): the primitive state `[rho, u, v, w, p]` (axial, supersonic) and `[k, omega]`
/// (defined with the solver: `bowshock_cfd::solver3::JetState3`, `Solver3::set_jet`).
pub use bowshock_cfd::solver3::JetState3;

impl Case3 {
    /// Sting grid of the 2D case ([`crate::engine::case::Case::new`] with `Wake::Sting`), the half domain with `24 << level` azimuthal cells
    /// and [`Numerics3::auto`].
    pub fn new(mach: f64, alpha: f64, body: BaseBody, fins: Option<Fins3>, level: u32) -> Case3 {
        let finned = fins.is_some();
        let grid = Grid3Params::half(sting_grid(&body, level), fins, Grid3Params::default_nk(level), level);
        let mut numerics = Numerics3::auto(mach, level);
        if finned {
            // the diagonal line solve (the GPU default) blows up on a finned half domain at incidence (M 3, alpha 6 at iteration ~110 on the GPU; ISSUES #24 on the CPU,
            // the tiny ANF): the block lines are stable on both and equal on both. Block for every finned case until the diagonal solve is fixed
            numerics.line_solve = Some(LineSolve3::Block);
        }
        Case3 {
            mach,
            alpha,
            body,
            grid,
            numerics,
            tier: Tier::Euler,
            progress: None,
            snapshots: None,
            jet: None,
            viscous: None,
            wall_thermal: WallThermal::Adiabatic,
            transition: None,
        }
    }

    /// This case with a motor jet from the centre of the base (the 2D [`crate::engine::case::Case::with_motor`]): the gap behind the base of the base-wake grid is
    /// split into the nozzle exit (`n_jet` rows at the axis, the mesher marks the two cell columns ahead of them `Part::Jet`, their `+i` faces `Wall::Jet`) and the
    /// annulus, from the wall spacing `dr_wall` of the grid: **call it after [`Case3::with_tier`]** (which sets that spacing). Needs the base-wake grid and a base
    /// without a sting.
    pub fn with_motor(mut self, jet: Jet) -> Result<Case3, Error> {
        let base = self.grid.base.as_mut().ok_or_else(|| Error::Grid("a motor needs the base-wake grid".to_string()))?;
        if base.n_sting != 0 {
            return Err(Error::Grid("a motor needs a base without a sting".to_string()));
        }
        if !(jet.r_exit > 0.0 && jet.r_exit < self.body.base_radius()) {
            return Err(Error::Grid("the nozzle exit must lie inside the base".to_string()));
        }
        let [rho, u, _, p] = jet.state;
        if !(rho > 0.0 && p > 0.0 && u >= (1.4 * p / rho).sqrt()) {
            return Err(Error::Grid("the nozzle exit state must be supersonic and axial (rho, p > 0, u >= c)".to_string()));
        }
        motor_rows(base, &self.body, &jet);
        self.jet = Some(jet);
        Ok(self)
    }

    /// A viscous case that stops on a relative drag tolerance `tol` over [`DRAG_REL_WINDOW`] iterations (the facade's `Analysis::drag_tolerance`); an Euler case is unchanged.
    pub fn with_drag_tolerance(mut self, tol: f64) -> Case3 {
        if self.viscous.is_some() {
            (self.numerics.rel_tol, self.numerics.window, self.numerics.stationary_min_iters, self.numerics.warm_min_iters) =
                (tol, DRAG_REL_WINDOW, DRAG_REL_WINDOW, DRAG_REL_WINDOW);
        }
        self
    }

    /// The jet state the solver gives the `Wall::Jet` faces (`None`: power off).
    pub fn jet_state3(&self) -> Option<JetState3> {
        self.jet.map(|j| JetState3 { state: [j.state[0], j.state[1], 0.0, 0.0, j.state[3]], turb: j.turb })
    }

    /// An axisymmetric body at zero incidence as a single-cell wedge of [`WEDGE_ANGLE`] on the base-wake grid of the 2D case
    /// ([`crate::engine::case::Case::new`] with `Wake::Base`), [`Numerics3::auto`] with the block line solve ([`Case3::on_wedge`]).
    pub fn wedge(mach: f64, body: BaseBody, level: u32) -> Case3 {
        let mut c = Case3::new(mach, 0.0, body, None, level);
        c.grid.base = Some(base_grid(&c.body, level));
        c.numerics = Numerics3::euler_wedge(mach, level);
        c.on_wedge()
    }

    /// A 2D engine case ([`crate::engine::case::Case::new`] with `Wake::Base`) as a wedge of [`WEDGE_ANGLE`]: its body, its base-wake grid and its sting grid,
    /// the numerics of the 2D case ([`Numerics3::from_2d`]: limiter, low-Mach flags, implicit parameters with the block lines, outflow) and, for a viscous case, its
    /// RANS parameters (`Case::viscous`: the grids are already wall-resolved) and motor jet (`Case::with_motor`: the grid already has the nozzle rows). `level` is the grid level (the 2D case does not keep it).
    pub fn wedge_of(case: &Case, level: u32) -> Case3 {
        let mut c = Case3::new(case.mach, 0.0, case.body.clone(), None, level);
        c.grid.sting = case.grid_sting.clone();
        c.grid.base = Some(case.grid_base.clone());
        c.numerics = Numerics3::from_2d(case, level);
        c.viscous = case.viscous;
        c.jet = case.motor;
        c.on_wedge()
    }

    /// This case on a single-cell wedge of [`WEDGE_ANGLE`], with the block line solve on every backend: [`LineSolve3::DiagonalCorrected`] (the GPU default of
    /// [`Numerics3::implicit_for`]) is non-finite on the wedge at every `cfl_max` (iteration 72 to 88, CPU f64 and GPU alike, ISSUES #24: its diagonal solve does
    /// not carry the source block of the wedge's pressure source), `Block` converges at `cfl_max` 10 in 350 to 1750 iterations. Whoever builds a wedge goes through
    /// here, so the choice cannot be forgotten.
    pub fn on_wedge(mut self) -> Case3 {
        self.grid.domain = Domain3::Wedge { dtheta: WEDGE_ANGLE };
        self.numerics.line_solve = Some(LineSolve3::Block);
        self
    }

    /// The fixed rule instead of the controller: [`Numerics3::cfl_control`] off and, on a viscous half domain or sector (not the wedge, which keeps the engine's 50 / 20), `cfl_max`
    /// at most [`VISCOUS_3D_CFL_MAX`]. For the CPU runner of a test that pins its numerics and for a run that must not roll back.
    pub fn without_cfl_control(mut self) -> Case3 {
        self.numerics.cfl_control = false;
        if self.viscous.is_some() && !matches!(self.grid.domain, Domain3::Wedge { .. }) {
            self.numerics.implicit.cfl_max = self.numerics.implicit.cfl_max.min(VISCOUS_3D_CFL_MAX);
        }
        self
    }

    /// A body, finned or not, on the base-wake grid as a half domain (incidence `alpha`, fins between the pitch plane and each other): `24 << level` azimuthal
    /// cells (`32 << level` on the lift tier), the walls resolved for `tier`.
    pub fn base_half(mach: f64, alpha: f64, body: BaseBody, fins: Option<Fins3>, tier: Tier, level: u32) -> Case3 {
        let mut c = Case3::new(mach, alpha, body, fins, level);
        c.grid.base = Some(base_grid(&c.body, level));
        c.with_tier(tier)
    }

    /// A body with `fins.count` fins at zero incidence as the sector `[0, pi/count]` on the base-wake grid (the product grid: the wake and the base are
    /// solved): `8 << level` azimuthal cells for [`Tier::Euler`], the fin stack for the wall-resolved tiers, the walls resolved for `tier`.
    pub fn sector(mach: f64, body: BaseBody, fins: Fins3, tier: Tier, level: u32) -> Case3 {
        let mut c = Case3::new(mach, 0.0, body, Some(fins), level);
        c.grid.base = Some(base_grid(&c.body, level));
        c.grid.domain = Domain3::Sector;
        c.grid.nk = 8usize << level;
        c.with_tier(tier)
    }

    /// This case's grids refined by the uniform factor `s` (`s = 1`: unchanged; opt-in, for grid-convergence studies, DESIGN3D item 47): every cell count of the meridian grids (axial,
    /// radial, gap rows, cells over the fin root chord and the span) is multiplied by `s` (rounded to an even count, at least 2; a count of 1, a diameter step, and 0 stay), the end-cell sizes
    /// and the wall spacings (`dx_tip`, `dr_wall`, `dr_gap`, the first cell `y1` of the fin stack) are divided by `s`, and the growth ratios of the fin stack become `1 + (g - 1) / s` (a
    /// fixed growth ratio would refine only the wall layer: the cell size at a distance `x` from a wall is about `(g - 1) x`). The geometric fans of the meridian grids keep their
    /// shape by themselves (the first cell and the cell count set the growth). The cells across the fin thickness (`fin_cells_across`) are an integer count that cannot follow a fractional `s`: they
    /// are left as they are (set them with [`Case3::with_fin_cells_across`]). Call it after [`Case3::with_tier`] / [`Case3::sector`]; the tier's `y+` becomes `y+ / s`. A motor's nozzle rows are not scaled.
    pub fn with_refinement(mut self, s: f64) -> Case3 {
        assert!(s > 0.0 && s.is_finite(), "refinement factor must be positive");
        let n = |c: usize| -> usize {
            if c <= 1 {
                c
            } else {
                (2 * (c as f64 * s / 2.0).round() as usize).max(2)
            }
        };
        let sting = &mut self.grid.sting;
        (sting.n_up, sting.n_nose, sting.n_ext, sting.nj) = (n(sting.n_up), n(sting.n_nose), n(sting.n_ext), n(sting.nj));
        sting.n_seg.iter_mut().for_each(|c| *c = n(*c));
        (sting.dx_tip, sting.dr_wall) = (sting.dx_tip / s, sting.dr_wall / s);
        if let Some(b) = &mut self.grid.base {
            (b.n_up, b.n_nose, b.n_wake, b.n_sting, b.n_gap, b.nj_out) = (n(b.n_up), n(b.n_nose), n(b.n_wake), n(b.n_sting), n(b.n_gap), n(b.nj_out));
            b.n_seg.iter_mut().for_each(|c| *c = n(*c));
            (b.dx_tip, b.dr_wall) = (b.dx_tip / s, b.dr_wall / s);
            b.dr_gap = b.dr_gap.map(|d| d / s);
        }
        if self.grid.fins.is_some() {
            self.grid.fin_chord_cells = Some(n(self.grid.fin_chord_cells.unwrap_or(32usize << self.grid.level)));
            self.grid.span_cells = self.grid.span_cells.map(n);
        }
        if let Some(st) = &mut self.grid.fin_stack {
            (st.y1, st.growth_bl, st.growth_out) = (st.y1 / s, 1.0 + (st.growth_bl - 1.0) / s, 1.0 + (st.growth_out - 1.0) / s);
        }
        self
    }

    /// This case with `n` cells across the thickness of its fins ([`Grid3Params::fin_cells_across`]; 1 is the one-cell layer of the default grids): the fin, the columns
    /// ahead of its leading edge and the base region behind a blunt trailing edge get `n` cells in `k`, a half fin on a symmetry plane (a sector) half of them. They are
    /// added to the cells of the fin stack, so the sector has `n / 2 - 1` more `k` cells than the default; a case without fins is unchanged.
    pub fn with_fin_cells_across(mut self, n: usize) -> Case3 {
        self.grid.fin_cells_across = n;
        self
    }

    /// This case with the thermal condition `thermal` on its no-slip walls (the viscous equations only: an Euler case has no heat flux). `Isothermal(t)` is the wall temperature
    /// over the free-stream static temperature; the ghost, the wall viscosity and the wall properties of the law of the wall follow `bowshock_cfd::sst::WallThermal`.
    pub fn with_wall_thermal(mut self, thermal: WallThermal) -> Case3 {
        self.wall_thermal = thermal;
        self
    }

    /// This case with a prescribed transition (`bowshock_cfd::transition3`, SST only): the walls of the body (and the sting) upstream of `x_tr` nose diameters from the tip are laminar
    /// (no eddy viscosity, no production of `k`, `omega`; the plain resolved-wall state), turbulent downstream; the fins are laminar over the fraction `fin_chord_fraction` of
    /// their local chord from the leading edge (`0`: turbulent from the leading edge, `1`: laminar over the whole fin). `with_transition(0.0, 0.0)` is the default, a fully turbulent run.
    pub fn with_transition(mut self, x_tr_body_calibers: f64, fin_chord_fraction: f64) -> Case3 {
        self.transition = Some(Transition { x_tr: x_tr_body_calibers, fin_frac: fin_chord_fraction });
        self
    }

    /// This case sized for `tier`: the first cell of the body walls on both meridian grids for the body `y+`, the fin stack for the fin `y+` (the wall-resolved
    /// tiers; the lift tier keeps uniform azimuthal cells, `32 << level` over a half domain), the roughness of the body and of the fins carried on the grid.
    pub fn with_tier(mut self, tier: Tier) -> Case3 {
        self.tier = tier;
        let (Some(flow), Some((y_body, y_fin))) = (tier.flow(), tier.y_plus()) else {
            return self;
        };
        let level = self.grid.level;
        let y1 = wall_spacing(&self.body, flow.re_c, flow.mach_max, y_body);
        let mut base = self.grid.base.clone().unwrap_or_else(|| base_grid(&self.body, level));
        if matches!(tier, Tier::Draft(..)) {
            draft_coarsen(&mut base, &mut self.grid.sting, &self.body);
        }
        resolve_walls(&mut base, &mut self.grid.sting, &self.body, y1);
        if self.grid.base.is_some() {
            self.grid.base = Some(base);
        }
        self.grid.roughness = PartRoughness { body: flow.ks_body, fin: flow.ks_fin };
        // a wall-resolved tier is a RANS run (SST, the automatic wall treatment): the equations follow the grid; `euler_equations` keeps the grid only
        self.viscous = Some(Viscous { re_c: flow.re_c, sutherland: flow.sutherland, turbulence: Turbulence::Sst, wall_i_start: 0, roughness: 0.0 });
        self.numerics = Numerics3::viscous(self.mach, level);
        if matches!(self.grid.domain, Domain3::Wedge { .. }) && self.mach < 1.0 {
            // one factored solve per iteration on a subsonic wedge (the supersonic wedges keep three: Arcas fins-off Mach 2.3 and TP-2005 Mach 1.65 at the medium level go
            // non-finite with one; the finned domains take two, below): same state, 20-25 % cheaper (DESIGN3D item 35)
            self.numerics.implicit.sub_iters = 1;
        }
        if self.grid.fins.is_some() {
            // two factored solves per iteration on a finned domain (GPU, G-F1 Coarse sector, 677 k cells, f32): Mach 0.5 6800 iterations in 59 s against 7100 in 87 s with three,
            // CD 0.24575 against 0.24584, fin friction 0.01342 against 0.01343; Mach 2 7000 iterations in 57 s against 6940 in 76 s, CD 0.34993 against 0.35017, fin friction
            // 0.00822 for both: 24-32 % less wall, the same state. One is not stable on the finned sector (see [`Numerics3::viscous`])
            self.numerics.implicit.sub_iters = 2;
        }
        if !matches!(self.grid.domain, Domain3::Wedge { .. }) {
            // the diagonally dominant factors (DDADI, omega 1) with the implicit face dissipation x 1.7 hold `cfl_max` 100 where the plain factors stop at 14-35 (GPU f32,
            // Draft example sector: CA within 0.5 % of the plain run's converged value after 800 / 850 iterations at Mach 0.5 / 2.3 against 3100 / 4150, no rollback,
            // +5.7 % per step on the G-F1 sector; CPU f64 viscous block: a boost of 0.7 removes the period-2 cycle of omega 1 up to CFL 300 at Mach 2, 0.2 at Mach 0.5)
            self.numerics.implicit.ddadi = Some(1.0);
            self.numerics.implicit.lhs_dis_boost = DDADI_DIS_BOOST;
            self.numerics.implicit.cfl_max = DDADI_CFL_MAX;
            // experiment (DESIGN3D item 60): `BOWSHOCK_DT_CAPK=<cfl_max>` takes the pseudo-time step with the `k` rate capped at `L_i + L_j` ([`DtMode::SumCapK`]) at that `cfl_max`
            if let Some(cfl) = std::env::var("BOWSHOCK_DT_CAPK").ok().and_then(|v| v.parse::<f64>().ok()) {
                self.numerics.implicit.dt_mode = DtMode::SumCapK;
                self.numerics.implicit.cfl_max = cfl;
            }
        }
        // the engine's `cfl_max` (100 with DDADI on the 3D domains; on the wedge 50 below Mach 1, 20 above) with the rollback controller of the runners
        // ([`Numerics3::cfl_control`]); the fixed rule of [`VISCOUS_3D_CFL_MAX`] is [`Case3::without_cfl_control`]
        match (&self.grid.fins, tier) {
            (Some(f), Tier::Draft(..)) => {
                // growth 1.35 / 1.6 in and outside the layer (1.25 / 1.35), 32 cells over the root chord (`DRAFT_CHORD_CELLS`; Coarse and Fine: `COARSE_CHORD_CELLS`)
                self.grid.fin_stack = Some(FinStack { growth_bl: 1.35, growth_out: 1.6, ..FinStack::from_flow(flow.re_c, flow.mach_max, f.root_chord, y_fin) });
                self.grid.fin_chord_cells = Some(DRAFT_CHORD_CELLS << level);
            }
            (Some(f), Tier::Coarse(..) | Tier::Fine(..)) => {
                self.grid.fin_stack = Some(FinStack::from_flow(flow.re_c, flow.mach_max, f.root_chord, y_fin));
                self.grid.fin_chord_cells = Some(COARSE_CHORD_CELLS << level);
            }
            (Some(f), _) => self.grid.fin_stack = Some(FinStack::from_flow(flow.re_c, flow.mach_max, f.root_chord, y_fin)),
            (None, Tier::Lift(_)) if matches!(self.grid.domain, Domain3::Half { .. }) => self.grid.nk = 32usize << level,
            _ => {}
        }
        self
    }

    /// The safe variant of the [`Tier::Draft`] grid for Mach numbers below 0.8 (DESIGN3D item 32: the subsonic base wake of the example body is at the edge of shedding, and the coarse gap rows,
    /// the small domain and the short wake push it over): the gap rows behind the base from the wall spacing, the full domain (10 L upstream and radially, 5 L downstream) and the full
    /// wake columns of the wall-resolved tiers, everything else (axial counts x 0.6 elsewhere, the wall functions, the fin stack) as the Draft tier. Other tiers: unchanged.
    pub fn draft_subsonic(mut self) -> Case3 {
        let Tier::Draft(flow, y) = self.tier else { return self };
        let Some(mut base) = self.grid.base.take() else { return self };
        let full = base_grid(&self.body, self.grid.level);
        (base.x_up, base.r_far, base.x_down, base.n_wake, base.dr_gap) = (full.x_up, full.r_far, full.x_down, full.n_wake, None);
        let y1 = wall_spacing(&self.body, flow.re_c, flow.mach_max, y);
        resolve_walls(&mut base, &mut self.grid.sting, &self.body, y1);
        self.grid.base = Some(base);
        self
    }

    /// The Euler equations on the grid of the tier (the fin-stack and wall-resolved grids of the Euler gates): drops the viscous terms and takes the numerics of
    /// [`Numerics3::auto`] (the block lines on a wedge).
    pub fn euler_equations(mut self) -> Case3 {
        self.viscous = None;
        self.numerics = Numerics3::auto(self.mach, self.grid.level);
        if matches!(self.grid.domain, Domain3::Wedge { .. }) || self.grid.fins.is_some() {
            self.numerics.line_solve = Some(LineSolve3::Block);
        }
        self
    }

    /// Free-stream state at the case's Mach number and incidence.
    pub fn freestream(&self) -> bowshock_cfd::gas3::Prim3 {
        freestream3(self.mach, self.alpha)
    }

    /// Cartesian grid, then the implicit CPU solver with the free stream everywhere.
    pub fn build_solver(&self) -> Result<Solver3, Error> {
        Ok(self.solver_on(body_grid3(&self.grid)?))
    }

    /// The implicit CPU solver of this case's numerics and free stream on `grid` (a grid the mesher did not build: the validation cases' wedges).
    pub fn solver_on(&self, grid: Grid3) -> Solver3 {
        let s = Solver3::new(grid, self.config3());
        self.with_equations(s)
    }

    /// [`Case3::solver_on`] with the axis map of `grid` already built (the grid cache shares it).
    pub fn solver_with_axis(&self, grid: Grid3, axis: bowshock_cfd::axis3::AxisMap) -> Solver3 {
        let s = Solver3::with_axis(grid, axis, self.config3());
        self.with_equations(s)
    }

    fn config3(&self) -> Config3 {
        let n = &self.numerics;
        Config3 {
            freestream: self.freestream(),
            cfl: 0.5,
            limiter: n.limiter,
            first_order_iters: n.first_order_iters,
            low_mach: n.low_mach,
            scheme: Scheme3::Implicit(n.implicit_for(false)),
        }
    }

    /// Switches the viscous (SST) terms of `s` on when the case is viscous: the sand-grain roughness of the walls from the grid's per-part values
    /// (`Grid3::wall_ks`), the outflow of the numerics (characteristic for a body: `RansOpts3::default` would impose a static pressure, right for plates only);
    /// gives the `Part::Jet` cells of a motor case their jet state (`Solver3::set_jet`, nothing without a motor).
    fn with_equations(&self, mut s: Solver3) -> Solver3 {
        s.set_jet(self.jet_state3());
        if let Some(v) = self.viscous {
            let ks = s.grid.wall_ks();
            let ks = if ks.iter().any(|&k| k > 0.0) { ks } else { Vec::new() };
            s.set_viscous_with(
                Some(v),
                RansOpts3 {
                    pressure_outflow: self.numerics.pressure_outflow,
                    ks,
                    wall_thermal: self.wall_thermal,
                    transition: self.transition,
                    ..RansOpts3::default()
                },
            );
        }
        s
    }

    /// Army-Navy Basic Finner body: 10 d long, 10 degree half-angle cone nose, cylindrical afterbody.
    #[doc(hidden)]
    pub fn anf_body() -> BaseBody {
        let nose = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
        let l_nose = nose.nose_length();
        BaseBody::cylinder(nose, 10.0 - l_nose)
    }

    /// ANF fins: four fins of 1 d chord and 1 d exposed span, thickness [`bowshock_reference::anf::FIN_THICKNESS_CAL`], flush with the
    /// base. Section (DESIGN3D item 38): a **wedge over the whole chord**, 0.08 cal thick at the trailing edge, with a leading-edge radius of 0.004 cal. The sources:
    /// Dupuis, DRDC TM 2002-136, p. 18 (second hand, from its optical reading by nrdptel/fusionspace-eridanus): four "1 cal x 1 cal" wedge fins, 0.08 caliber thick at the
    /// trailing edge, leading-edge and nose radius 0.004 caliber; Bolz and Nicolaides, BRL Report 711 (1949), p. 29: wedge fins 16 % thick; Abiza et al. (p. 3): four 1 x 1
    /// caliber fins with sharp leading edges and 0.08 caliber trailing-edge thickness. As a `FinProfile::Custom` section (fractions of the maximum thickness): `[[0,0.1],
    /// [0.5,0.55],[1,1]]`, the leading-edge half thickness `0.1 t/2 = 0.004` cal (the three points make the section one wedge for `fins::section_edges`).
    #[doc(hidden)]
    pub fn anf_fins() -> Fins3 {
        let profile = FinProfile::Custom { section: vec![[0.0, 0.1], [0.5, 0.55], [1.0, 1.0]] };
        Fins3 {
            count: 4,
            root_chord: 1.0,
            tip_chord: 1.0,
            span: 1.0,
            sweep: 0.0,
            thickness: bowshock_reference::anf::FIN_THICKNESS_CAL,
            te_to_base: 0.0,
            profile,
            planform: None,
        }
    }

    /// Coarse ANF-like case for tests: [`Case3::anf_body`] with [`Case3::anf_fins`], level 0, `nk = 8`, 16 cells over the fin chord and the sting grid counts
    /// `n_up 10, n_nose 16, n_seg [12], n_ext 10, nj 24`. `max_iter` 3000.
    #[doc(hidden)]
    pub fn tiny_anf(mach: f64, alpha: f64) -> Case3 {
        let mut c = Case3::new(mach, alpha, Case3::anf_body(), Some(Case3::anf_fins()), 0);
        let s = &mut c.grid.sting;
        s.n_up = 10;
        s.n_nose = 16;
        s.n_seg = vec![12];
        s.n_ext = 10;
        s.nj = 24;
        c.grid.nk = 8;
        c.grid.fin_chord_cells = Some(16);
        c.numerics.max_iter = 3000;
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `with_refinement(1)` changes nothing; `with_refinement(2)` doubles every cell count of the base-wake grid and of the fin chord, halves the wall spacings and the stack's first
    /// cell and takes the excess of the stack's growth over 1 to half (DESIGN3D item 47).
    #[test]
    fn refinement_scales_counts_spacings_and_growths_uniformly() {
        let flow = WallFlow { re_c: 7.0e5, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
        let base = Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(flow), 0);
        let same = base.clone().with_refinement(1.0);
        let (b0, b1) = (base.grid.base.as_ref().unwrap(), same.grid.base.as_ref().unwrap());
        assert_eq!((b0.n_up, b0.n_nose, &b0.n_seg, b0.n_wake, b0.n_gap, b0.nj_out), (b1.n_up, b1.n_nose, &b1.n_seg, b1.n_wake, b1.n_gap, b1.nj_out));
        assert_eq!(same.grid.fin_stack, base.grid.fin_stack);
        assert_eq!(same.grid.fin_chord_cells, Some(64));
        let fine = base.clone().with_refinement(2.0);
        let b2 = fine.grid.base.as_ref().unwrap();
        assert_eq!(
            (b2.n_up, b2.n_nose, b2.n_seg[0], b2.n_wake, b2.n_gap, b2.nj_out),
            (2 * b0.n_up, 2 * b0.n_nose, 2 * b0.n_seg[0], 2 * b0.n_wake, 2 * b0.n_gap, 2 * b0.nj_out)
        );
        assert_eq!(fine.grid.fin_chord_cells, Some(128));
        assert!((b2.dr_wall - 0.5 * b0.dr_wall).abs() < 1e-18 && (b2.dx_tip - 0.5 * b0.dx_tip).abs() < 1e-18);
        let (s0, s2) = (base.grid.fin_stack.unwrap(), fine.grid.fin_stack.unwrap());
        assert!((s2.y1 - 0.5 * s0.y1).abs() < 1e-18);
        assert!((s2.growth_bl - 1.0 - 0.5 * (s0.growth_bl - 1.0)).abs() < 1e-12 && (s2.growth_out - 1.0 - 0.5 * (s0.growth_out - 1.0)).abs() < 1e-12);
        assert_eq!(fine.grid.fin_cells_across, base.grid.fin_cells_across);
    }

    /// Prescribed transition is opt-in: the default case and `with_transition(0, 0)` hand the solver no intermittency field (fully turbulent, bit-identical to the solver without the
    /// feature); `with_transition(x_tr, f)` gives the near-wall cells of the body upstream of `x_tr` calibers `gamma = 0` and those downstream `1`, `RansOpts3::transition`.
    #[test]
    fn transition_is_opt_in_and_reaches_the_solver() {
        let flow = WallFlow { re_c: 7.0e5, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
        let base = Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(flow), 0);
        assert_eq!(base.transition, None);
        let gamma_of = |c: &Case3| {
            let s = c.build_solver().unwrap();
            let r = s.rans().expect("viscous");
            let near: Vec<(f64, f64)> =
                (0..s.grid.vol.len()).filter(|&i| r.dwall[i] < 0.02).map(|i| (s.grid.xc[i][0], r.gamma.get(i).copied().unwrap_or(1.0))).collect();
            (r.gamma.is_empty(), near)
        };
        assert!(gamma_of(&base).0);
        assert!(gamma_of(&base.clone().with_transition(0.0, 0.0)).0);
        let (empty, near) = gamma_of(&base.with_transition(2.0, 0.0));
        assert!(!empty);
        assert!(near.iter().filter(|&&(x, _)| x < 1.5).all(|&(_, g)| g == 0.0), "the body wall cells upstream of the trip are laminar");
        assert!(near.iter().filter(|&&(x, _)| x > 2.5 && x < 8.5).all(|&(_, g)| g == 1.0), "and the ones downstream turbulent");
        assert!(near.iter().any(|&(x, _)| x < 1.5) && near.iter().any(|&(x, _)| x > 2.5 && x < 8.5));
    }

    /// Every viscous 3D domain (half domain, sector) starts at [`DDADI_CFL_MAX`] with the DDADI line solve and the runners' rollback controller on; the wedge at the engine's
    /// `cfl_max` (50 below Mach 1, 20 above) with the plain factors; [`Case3::without_cfl_control`] takes the fixed [`VISCOUS_3D_CFL_MAX`] on a half domain or a sector (not the
    /// wedge), Euler keeps its own.
    #[test]
    fn viscous_3d_grids_start_at_the_engine_cfl_with_the_controller() {
        let flow = WallFlow { re_c: 7.0e5, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
        let cfl = |c: &Case3| c.numerics.implicit.cfl_max;
        for (mach, engine) in [(0.8, 50.0), (2.0, 20.0)] {
            let half = Case3::base_half(mach, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), Tier::Lift(flow), 0);
            let finless = Case3::base_half(mach, 0.0, Case3::anf_body(), None, Tier::Lift(flow), 0);
            let sectors: Vec<Case3> =
                [Tier::Coarse(flow), Tier::draft(flow)].into_iter().map(|tier| Case3::sector(mach, Case3::anf_body(), Case3::anf_fins(), tier, 0)).collect();
            for c in [&half, &finless].into_iter().chain(&sectors) {
                assert_eq!(cfl(c), DDADI_CFL_MAX, "Mach {mach}");
                assert_eq!((c.numerics.implicit.ddadi, c.numerics.implicit.lhs_dis_boost), (Some(1.0), DDADI_DIS_BOOST), "Mach {mach}");
                assert!(c.numerics.cfl_control);
                let fixed = c.clone().without_cfl_control();
                assert_eq!((cfl(&fixed), fixed.numerics.cfl_control), (VISCOUS_3D_CFL_MAX, false), "Mach {mach}");
            }
            let on_wedge = Case3::wedge(mach, Case3::anf_body(), 0).with_tier(Tier::Coarse(flow));
            assert_eq!(cfl(&on_wedge), engine, "wedge at Mach {mach}");
            assert_eq!(on_wedge.numerics.implicit.ddadi, None, "the wedge keeps the plain factors");
            assert_eq!(cfl(&on_wedge.clone().without_cfl_control()), engine, "the wedge keeps the engine's limit");
            let euler = Case3::base_half(mach, 0.0, Case3::anf_body(), Some(Case3::anf_fins()), Tier::Euler, 0);
            assert_eq!(cfl(&euler), Numerics3::auto(mach, 0).implicit.cfl_max, "Euler half domain at Mach {mach}");
        }
    }
}
