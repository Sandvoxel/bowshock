//! A single (Mach number, incidence) 3D Euler case: sting-mounted body with optional thick fins, grid parameters and numerics, and the
//! CPU solver built from them. Lengths are in nose diameters (`d = 1`).

use crate::engine::body_grid::{BaseBody, Body, Nose};
use crate::engine::case::sting_grid;
use crate::engine::grid3::{body_grid3, Fins3, Grid3Params};
use crate::error::Error;
use crate::progress::ProgressSink;
use crate::spec::FinProfile;
use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::freestream3;
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};

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
    /// Explicit 3D line solve overriding the backend default (`None`: [`Numerics3::implicit_for`]).
    pub line_solve: Option<LineSolve3>,
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
            line_solve: None,
        }
    }

    /// The implicit parameters for a backend with the line solve resolved: `line_solve` if set, else the backend default.
    ///
    /// The default on the GPU is the diagonalised form with one defect-correction pass against the block operator
    /// ([`LineSolve3::DiagonalCorrected`]; `sub_iters` is then not used). It was measured on the GPU only: on the ANF case (level 0, f32, RTX 3090) a step costs 1935 us instead of
    /// 3151 us, the iterations and CN equal those of the block scheme (CN within 0.01 %) at Mach 0.5, 0.8, 2 and 3, 1.56x faster to the stop rule.
    /// The f64 CPU solver with it diverges on the tiny ANF grid at `cfl_max` 10 (it converges at 5), so the CPU default is the block solve.
    pub fn implicit_for(&self, gpu: bool) -> ImplicitParams {
        let default = if gpu { LineSolve3::DiagonalCorrected } else { LineSolve3::Block };
        ImplicitParams { line_solve: self.line_solve.unwrap_or(default), ..self.implicit }
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
    /// Where the run reports its progress and can be stopped; `None`: silent.
    pub progress: Option<ProgressSink>,
}

impl Case3 {
    /// Sting grid of the 2D case ([`crate::engine::case::Case::new`] with `Wake::Sting`), `24 << level` azimuthal cells and
    /// [`Numerics3::auto`].
    pub fn new(mach: f64, alpha: f64, body: BaseBody, fins: Option<Fins3>, level: u32) -> Case3 {
        let sting = sting_grid(&body, level);
        let grid = Grid3Params { sting, fins, nk: Grid3Params::default_nk(level), level, fin_chord_cells: None, span_cells: None };
        Case3 { mach, alpha, body, grid, numerics: Numerics3::auto(mach, level), progress: None }
    }

    /// Free-stream state at the case's Mach number and incidence.
    pub fn freestream(&self) -> bowshock_cfd::gas3::Prim3 {
        freestream3(self.mach, self.alpha)
    }

    /// Cartesian grid, then the implicit CPU solver with the free stream everywhere.
    pub fn build_solver(&self) -> Result<Solver3, Error> {
        let grid = body_grid3(&self.grid)?;
        let n = &self.numerics;
        let cfg = Config3 {
            freestream: self.freestream(),
            cfl: 0.5,
            limiter: n.limiter,
            first_order_iters: n.first_order_iters,
            low_mach: n.low_mach,
            scheme: Scheme3::Implicit(n.implicit_for(false)),
        };
        Ok(Solver3::new(grid, cfg))
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
