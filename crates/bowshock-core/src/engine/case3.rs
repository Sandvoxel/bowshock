//! A single (Mach number, incidence) 3D Euler case: sting-mounted body with optional thick fins, grid parameters and numerics, and the
//! CPU solver built from them. Lengths are in nose diameters (`d = 1`).

use crate::engine::body_grid::{BaseBody, Body, Nose};
use crate::engine::case::{base_grid, motor_rows, resolve_walls, sting_grid, wall_spacing, Case, Jet};
use crate::engine::field3::Snapshots3;
use crate::engine::grid3::{body_grid3, Domain3, FinStack, Fins3, Grid3Params, WEDGE_ANGLE};
use crate::error::Error;
use crate::progress::ProgressSink;
use crate::spec::FinProfile;
use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::freestream3;
use bowshock_cfd::grid3::{Grid3, PartRoughness};
use bowshock_cfd::implicit::{DtMode, ImplicitParams, LineSolve3};
use bowshock_cfd::solver::{Bc, Scheme, Turbulence, Viscous};
use bowshock_cfd::solver3::{Config3, RansOpts3, Scheme3, Solver3};

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
    /// when the means of CA (and, at incidence, of CN and of the moment about the nose) over two consecutive windows agree within `mean_tol` (relative, plus 1e-4 absolute) and
    /// both windows lie after iteration `mean_from`; the result is then the mean of the second window. For the oscillating subsonic base wake, whose period (50 to 220 iterations)
    /// the range rule cannot see.
    pub mean_window: usize,
    pub mean_from: usize,
    pub mean_tol: f64,
    /// Explicit 3D line solve overriding the backend default (`None`: [`Numerics3::implicit_for`]).
    pub line_solve: Option<LineSolve3>,
    /// Static-pressure outflow `p = p_inf` at the last `i` plane instead of the characteristic ghost (the plates of the viscous gates; the body cases of the 2D
    /// engine, and so these, use the characteristic outflow: `false`). Read by the viscous solver only.
    pub pressure_outflow: bool,
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
            mean_tol: 5e-3,
            line_solve: None,
            pressure_outflow: false,
        }
    }

    /// The settings of a wall-resolved RANS run, as the 2D engine's `Numerics::viscous` has them: van Albada below Mach 0.85 and minmod above, the low-Mach
    /// correction and (below Mach 1) the low-Mach preconditioning of the implicit left-hand side, `cfl_max` 50 (20 from Mach 1: the supersonic wake diverges at 50),
    /// `DtMode::Sum`, 200 first-order iterations, the **block** lines (the diagonal solve is unmeasured on viscous wedges and stalls with the preconditioning on plates),
    /// the characteristic outflow. The stationary rule waits 1000 iterations and a window of 1000 (friction settles slowly).
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
            line_solve: Some(LineSolve3::Block),
            pressure_outflow: false,
        }
    }

    /// The numerics of a 2D engine case on the wedge: its limiter, low-Mach flags, first-order iterations, implicit parameters (with the block lines) and outflow;
    /// the stop rule is [`Numerics3::auto`]'s for an Euler case and [`Numerics3::viscous`]'s for a viscous one.
    pub fn from_2d(case: &Case, level: u32) -> Numerics3 {
        let mut n = if case.viscous.is_some() { Numerics3::viscous(case.mach, level) } else { Numerics3::auto(case.mach, level) };
        n.limiter = case.numerics.limiter;
        n.low_mach = case.numerics.low_mach;
        n.first_order_iters = case.numerics.first_order_iters;
        n.max_iter = case.numerics.max_iter;
        if let Scheme::Implicit(p) = case.numerics.scheme {
            n.implicit = ImplicitParams { line_solve: LineSolve3::Block, ..p };
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

/// What a 3D grid is resolved for: the wall spacing of the body and of the fin sides, and the azimuthal policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tier {
    /// Inviscid: no wall spacing, uniform azimuthal cells.
    Euler,
    /// `Resolution::Coarse` of a wall-resolved case: body `y+` ~ 1, fin sides `y+` ~ 15 (automatic wall treatment), the fin stack of [`FinStack`].
    Coarse(WallFlow),
    /// Body and fin sides `y+` ~ 1.
    Fine(WallFlow),
    /// The lift tier of a half domain: body and fins `y+` ~ 60 (wall functions); without fins uniform azimuthal cells, `32 << level` over the half domain, with fins the
    /// fin stack of [`FinStack`] at `y+` 60 (a uniform azimuth puts the first cell of a fin side at thousands of wall units).
    Lift(WallFlow),
}

impl Tier {
    /// The flow the walls are sized for (`None` for [`Tier::Euler`]).
    pub fn flow(&self) -> Option<&WallFlow> {
        match self {
            Tier::Euler => None,
            Tier::Coarse(f) | Tier::Fine(f) | Tier::Lift(f) => Some(f),
        }
    }

    /// Target `y+` of the first cell on the body and on the fin sides (`None` for [`Tier::Euler`]).
    pub fn y_plus(&self) -> Option<(f64, f64)> {
        match self {
            Tier::Euler => None,
            Tier::Coarse(_) => Some((1.0, 15.0)),
            Tier::Fine(_) => Some((1.0, 1.0)),
            Tier::Lift(_) => Some((60.0, 60.0)),
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
        Case3 { mach, alpha, body, grid, numerics, tier: Tier::Euler, progress: None, snapshots: None, jet: None, viscous: None }
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

    /// The jet state the solver gives the `Wall::Jet` faces (`None`: power off).
    pub fn jet_state3(&self) -> Option<JetState3> {
        self.jet.map(|j| JetState3 { state: [j.state[0], j.state[1], 0.0, 0.0, j.state[3]], turb: j.turb })
    }

    /// An axisymmetric body at zero incidence as a single-cell wedge of [`WEDGE_ANGLE`] on the base-wake grid of the 2D case
    /// ([`crate::engine::case::Case::new`] with `Wake::Base`), [`Numerics3::auto`] with the block line solve ([`Case3::on_wedge`]).
    pub fn wedge(mach: f64, body: BaseBody, level: u32) -> Case3 {
        let mut c = Case3::new(mach, 0.0, body, None, level);
        c.grid.base = Some(base_grid(&c.body, level));
        c.on_wedge()
    }

    /// A 2D engine case ([`crate::engine::case::Case::new`] with `Wake::Base`) as a wedge of [`WEDGE_ANGLE`]: its body, its base-wake grid and its sting grid,
    /// the numerics of the 2D case ([`Numerics3::from_2d`]: limiter, low-Mach flags, implicit parameters with the block lines, outflow) and, for a viscous case, its
    /// RANS parameters (`Case::viscous`: the grids are already wall-resolved). `level` is the grid level (the 2D case does not keep it).
    pub fn wedge_of(case: &Case, level: u32) -> Case3 {
        let mut c = Case3::new(case.mach, 0.0, case.body.clone(), None, level);
        c.grid.sting = case.grid_sting.clone();
        c.grid.base = Some(case.grid_base.clone());
        c.numerics = Numerics3::from_2d(case, level);
        c.viscous = case.viscous;
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
        resolve_walls(&mut base, &mut self.grid.sting, &self.body, y1);
        if self.grid.base.is_some() {
            self.grid.base = Some(base);
        }
        self.grid.roughness = PartRoughness { body: flow.ks_body, fin: flow.ks_fin };
        // a wall-resolved tier is a RANS run (SST, the automatic wall treatment): the equations follow the grid; `euler_equations` keeps the grid only
        self.viscous = Some(Viscous { re_c: flow.re_c, sutherland: flow.sutherland, turbulence: Turbulence::Sst, wall_i_start: 0, roughness: 0.0 });
        self.numerics = Numerics3::viscous(self.mach, level);
        match (&self.grid.fins, tier) {
            (Some(f), _) => self.grid.fin_stack = Some(FinStack::from_flow(flow.re_c, flow.mach_max, f.root_chord, y_fin)),
            (None, Tier::Lift(_)) if matches!(self.grid.domain, Domain3::Half { .. }) => self.grid.nk = 32usize << level,
            _ => {}
        }
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
    /// gives the `Part::Jet` cells of a motor case the jet state.
    fn with_equations(&self, mut s: Solver3) -> Solver3 {
        if let Some(v) = self.viscous {
            let ks = s.grid.wall_ks();
            let ks = if ks.iter().any(|&k| k > 0.0) { ks } else { Vec::new() };
            s.set_viscous_with(Some(v), RansOpts3 { pressure_outflow: self.numerics.pressure_outflow, ks, ..RansOpts3::default() });
        }
        // phys3d hook: the motor jet patch (`Part::Jet` cells of the grid, `Solver3::set_jet`); nothing without a motor
        s.set_jet(self.jet_state3());
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
    /// base. Section: sharp leading edge and a blunt trailing edge of the full thickness (0.08 cal), as Abiza et al. (p. 3) describe
    /// them. The source gives no bevel angle: the `FinProfile::Custom` section `[[0,0],[0.1,1],[1,1]]` (a wedge over the first
    /// 0.1 chord, about 22 degrees per side, floored at 25 % of the half thickness) is **assumed**, and CN_alpha at M >= 2.35 is
    /// sensitive to it.
    #[doc(hidden)]
    pub fn anf_fins() -> Fins3 {
        let profile = FinProfile::Custom { section: vec![[0.0, 0.0], [0.1, 1.0], [1.0, 1.0]] };
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
