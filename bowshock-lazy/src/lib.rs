//! Experiment: the explicit solver of `bowshock-core` (`Solver::eval_residual` + `rk3_step`: MUSCL reconstruction, HLLC /
//! Rusanov blend with the shock sensor, optional low-Mach correction, characteristic and pressure-outflow far fields,
//! SSP-RK3 with local time stepping) written in plain Luma array ops, no custom kernels.
//!
//! The solver is in `body.rs`, included once per precision:
//!
//! * [`f32s`]: `Array<f32>` everywhere (compare `bowshock_gpu::Precision::F32`).
//! * [`df32s`]: the state, the residual and the time step in double-float `Array<Df32>` (compare
//!   `bowshock_gpu::Precision::Extended`); the state is a `Df32` record through the loops.
//!
//! Grids with a solid block, the implicit scheme and viscous terms are not covered.

/// How much of the order of evaluation is left to Luma's fusion engine (`LazySolver::staging`, see README).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Staging {
    /// No `cache()` anywhere: the whole step is one lazy expression and Luma's fusion decides where to cut.
    Fusion,
    /// The padded primitive record (state with its ghost layers) is evaluated once per residual evaluation.
    Lines,
    /// Additionally the face fluxes are evaluated: three kernels per Runge-Kutta stage (padded state, i fluxes, j fluxes),
    /// the stage update and the residual fuse into the padded-state kernel of the next stage.
    Fluxes,
    /// Additionally the reconstructed states and the shock weight are evaluated before the Riemann solvers.
    Split,
}

/// Single precision.
pub mod f32s {
    pub type S = f32;

    fn c(x: f64) -> S {
        x as f32
    }

    /// Host data (row-major, `i` fastest) as an array of `shape`.
    fn up(ctx: &Context, data: &[f64], shape: impl Into<luma::Shape>) -> Result<A> {
        let v: Vec<f32> = data.iter().map(|&x| x as f32).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    /// Measured fastest (README, speed table): `Fluxes` is 14 launches per step, `Fusion` (no `cache()`) 67-91.
    const DEFAULT_STAGING: Staging = Staging::Fluxes;

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(f64::from).collect())
    }

    /// The density residual of every step, recorded on the GPU.
    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until.record("residual", |s: &State| s[4].clone())
    }

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        run.history("residual").expect("residual history").iter().map(|&r| f64::from(r)).collect()
    }

    include!("body.rs");
}

/// Double-float state, residual and time step.
pub mod df32s {
    use luma::Df32;

    pub type S = Df32;

    fn c(x: f64) -> S {
        Df32::from(x)
    }

    fn up(ctx: &Context, data: &[f64], shape: impl Into<luma::Shape>) -> Result<A> {
        let v: Vec<Df32> = data.iter().map(|&x| Df32::from(x)).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    /// Measured fastest (README, speed table): `Fluxes` is 14 launches per step, `Fusion` (no `cache()`) 67-91.
    const DEFAULT_STAGING: Staging = Staging::Fluxes;

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(|d| d.to_f64()).collect())
    }

    /// The density residual of every step, recorded on the GPU as two `f32` values: the history of `iterate_until_all` is
    /// `f32`, so the double-float residual `hi + lo` is recorded as its high word and its low word.
    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until
            .record("residual", |s: &State| s[4].to_f32())
            .record("residual_lo", |s: &State| (&s[4] - s[4].to_f32().to_df32()).to_f32())
    }

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        let (hi, lo) = (run.history("residual").expect("residual history"), run.history("residual_lo").expect("residual history"));
        hi.iter().zip(lo).map(|(&h, &l)| f64::from(h) + f64::from(l)).collect()
    }

    include!("body.rs");
}
