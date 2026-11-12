//! Experiment: the explicit solver of `bowshock-core` (`Solver::eval_residual` + `rk3_step`: MUSCL reconstruction, HLLC /
//! Rusanov blend with the shock sensor, optional low-Mach correction, characteristic and pressure-outflow far fields,
//! SSP-RK3 with local time stepping) written in plain Luma array ops, no custom kernels.
//!
//! The solver is in `body.rs`, included once per precision:
//!
//! * [`f32s`]: `Array<f32>` everywhere (compare `bowshock_gpu::Precision::F32`).
//! * [`df32s`]: the state, the residual and the time step in double-float `Array<Df32>` (compare
//!   `bowshock_gpu::Precision::Extended`); the state is stored between steps as an `f32` pair `hi + lo`.
//!
//! Grids with a solid block, the implicit scheme and viscous terms are not covered.

/// How much of the order of evaluation is left to Luma's fusion engine (`LazySolver::staging`, see README).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Staging {
    /// Only the grid-level arrays are evaluated explicitly: primitives, the ghost layers of every boundary, the grid lines,
    /// the face fluxes and the residual.
    Fusion,
    /// Additionally the ghost layers of a direction are one array, and the state of every Runge-Kutta stage is evaluated; the
    /// face flux is still one expression.
    Lines,
    /// Additionally the reconstructed states and the shock weight are evaluated before the Riemann solvers (two kernels per
    /// direction instead of one).
    Split,
}

/// Single precision.
pub mod f32s {
    pub type S = f32;

    fn c(x: f64) -> S {
        x as f32
    }

    /// Host data (row-major, `i` fastest) as an array of `shape`.
    fn up(ctx: &Context, data: &[f64], shape: [usize; 2]) -> Result<A> {
        let v: Vec<f32> = data.iter().map(|&x| x as f32).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    fn cache(a: &A) -> A {
        a.cache()
    }

    /// Measured fastest for the f32 kernels (README, speed table): the face flux as one kernel.
    const DEFAULT_STAGING: Staging = Staging::Lines;

    fn aabs(a: &A) -> A {
        a.abs()
    }

    fn alt(a: &A, b: &A) -> Array<u32> {
        a.lt(b)
    }

    fn amin(a: &A, b: &A) -> A {
        a.minimum(b)
    }

    fn amax(a: &A, b: &A) -> A {
        a.maximum(b)
    }

    /// `[rho, rho u, rho v, E, residual]`.
    pub type State = [Array<f32>; 5];

    fn unpack(s: &State) -> Rec {
        rec(|k| dup(&s[k]))
    }

    fn pack(u: Rec, rn: A) -> State {
        let [u0, u1, u2, u3] = u;
        [u0, u1, u2, u3, rn]
    }

    fn upload_state(ctx: &Context, u: &[[f64; 4]], shape: [usize; 2]) -> Result<State> {
        let comp = |k: usize| up(ctx, &u.iter().map(|q| q[k]).collect::<Vec<_>>(), shape);
        Ok([comp(0)?, comp(1)?, comp(2)?, comp(3)?, Array::zeros_on(ctx, 1)?])
    }

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(f64::from).collect())
    }

    fn download_state(s: &State) -> Result<Vec<[f64; 4]>> {
        let v: Vec<Vec<f64>> = (0..4).map(|k| to_f64(&s[k])).collect::<Result<_>>()?;
        Ok((0..v[0].len()).map(|c| [v[0][c], v[1][c], v[2][c], v[3][c]]).collect())
    }

    fn rn_arrays(s: &State) -> Vec<Array<f32>> {
        vec![dup(&s[4])]
    }

    fn rn_host(a: &[Array<f32>]) -> Result<f64> {
        Ok(f64::from(a[0].item()?))
    }

    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until.record("residual", |s: &State| &s[4] * 1.0)
    }

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        run.history("residual").expect("residual history").iter().map(|&r| f64::from(r)).collect()
    }

    fn replay(init: State, steps: usize, step: impl Fn(&State) -> State) -> State {
        luma::iterate_all(init.each_ref(), steps, step)
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

    fn up(ctx: &Context, data: &[f64], shape: [usize; 2]) -> Result<A> {
        let v: Vec<Df32> = data.iter().map(|&x| Df32::from(x)).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    fn cache(a: &A) -> A {
        a.cache()
    }

    /// Measured fastest for the double-float kernels (README, speed table): the face flux as two kernels.
    const DEFAULT_STAGING: Staging = Staging::Split;

    // Luma bug workaround (see README, "Luma issues", no. 1): a double-float kernel whose only library calls are
    // comparisons (`abs`, `minimum`, `maximum`, `lt` of two arrays) and that has no scalar operand fails to bind its
    // (unused) guard uniform. Every comparison below therefore carries an arithmetic operation or a scalar compare.
    fn aabs(a: &A) -> A {
        select(&a.lt(c(0.0)), -a, a)
    }

    fn alt(a: &A, b: &A) -> Array<u32> {
        (a - b).lt(c(0.0))
    }

    fn amin(a: &A, b: &A) -> A {
        select(&alt(a, b), a, b)
    }

    fn amax(a: &A, b: &A) -> A {
        select(&alt(a, b), b, a)
    }

    /// `[rho, rho u, rho v, E]` high words, the same low words, residual high and low word.
    pub type State = [Array<f32>; 10];

    /// `hi + lo` as a double-float (an exact two-sum).
    fn unpack(s: &State) -> Rec {
        rec(|k| s[k].to_df32() + s[4 + k].to_df32())
    }

    /// `hi = round_f32(x)`, `lo = x - hi`.
    ///
    /// Luma bug workaround (README, "Luma issues", no. 2): `hi` must be a buffer before `lo` is computed. In one fused kernel
    /// `x - x.to_f32().to_df32()` evaluates to zero (the cast `x.hi + x.lo` is not guarded against reassociation).
    fn split(x: &A) -> (Array<f32>, Array<f32>) {
        let hi = x.to_f32().cache();
        let lo = (x - hi.to_df32()).to_f32();
        (hi, lo)
    }

    fn pack(u: Rec, rn: A) -> State {
        let (h0, l0) = split(&u[0]);
        let (h1, l1) = split(&u[1]);
        let (h2, l2) = split(&u[2]);
        let (h3, l3) = split(&u[3]);
        let (rh, rl) = split(&rn);
        [h0, h1, h2, h3, l0, l1, l2, l3, rh, rl]
    }

    fn upload_state(ctx: &Context, u: &[[f64; 4]], shape: [usize; 2]) -> Result<State> {
        let hi = |k: usize| -> Vec<f32> { u.iter().map(|q| q[k] as f32).collect() };
        let lo = |k: usize| -> Vec<f32> { u.iter().map(|q| (q[k] - f64::from(q[k] as f32)) as f32).collect() };
        let a = |v: Vec<f32>| Array::from_slice_on(ctx, &v, shape);
        Ok([a(hi(0))?, a(hi(1))?, a(hi(2))?, a(hi(3))?, a(lo(0))?, a(lo(1))?, a(lo(2))?, a(lo(3))?, Array::zeros_on(ctx, 1)?, Array::zeros_on(ctx, 1)?])
    }

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(|d| d.to_f64()).collect())
    }

    fn download_state(s: &State) -> Result<Vec<[f64; 4]>> {
        let v: Vec<(Vec<f32>, Vec<f32>)> = (0..4).map(|k| Ok((s[k].to_vec()?, s[4 + k].to_vec()?))).collect::<Result<_>>()?;
        Ok((0..v[0].0.len()).map(|c| std::array::from_fn(|k| f64::from(v[k].0[c]) + f64::from(v[k].1[c]))).collect())
    }

    fn rn_arrays(s: &State) -> Vec<Array<f32>> {
        vec![dup(&s[8]), dup(&s[9])]
    }

    fn rn_host(a: &[Array<f32>]) -> Result<f64> {
        Ok(f64::from(a[0].item()?) + f64::from(a[1].item()?))
    }

    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until.record("residual", |s: &State| &s[8] * 1.0).record("residual_lo", |s: &State| &s[9] * 1.0)
    }

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        let (hi, lo) = (run.history("residual").expect("residual history"), run.history("residual_lo").expect("residual history"));
        hi.iter().zip(lo).map(|(&h, &l)| f64::from(h) + f64::from(l)).collect()
    }

    fn replay(init: State, steps: usize, step: impl Fn(&State) -> State) -> State {
        luma::iterate_all(init.each_ref(), steps, step)
    }

    include!("body.rs");
}
