//! Parameters of the line-implicit pseudo-time stepping ([`ImplicitParams`], [`DtMode`], [`LineSolve3`]) shared by the CPU solver ([`crate::implicit3`]), the
//! low-Mach preconditioning ([`crate::lowmach5`]), the GPU ports and the engine: what is not part of either solver's kernels.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtMode {
    /// `dt = CFL A / (L_i + L_j)` (the explicit definition).
    Sum,
    /// `dt = CFL A / (2 min(L_i, L_j))`: sized by the weaker direction.
    MinDirection,
    /// 3D only: `dt = CFL V / (L_i + L_j + min(L_k, L_i + L_j))`. The sum, but the `k` rate (the azimuthal one of a finned sector, which clusters cells along every `k` line near the
    /// axis and the fins) counts at most as much as the other two together: the `k` direction is line-implicit like the others, and a sliver cell `r dtheta_min` wide must not set the
    /// step of the whole wake cell. Unlike [`DtMode::MinDirection`] it never releases both directions at once (the step is within a factor 3/2 of the sum's when `L_k >= L_i + L_j`
    /// and equal to it otherwise). 2D (no `k`): the same as `Sum`.
    SumCapK,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImplicitParams {
    /// Pseudo-time CFL: starts at `cfl0` and is multiplied by `growth` every iteration up to `cfl_max`.
    pub cfl0: f64,
    pub cfl_max: f64,
    pub growth: f64,
    pub dt_mode: DtMode,
    /// Solve block-tridiagonal lines (`true`) or drop the off-diagonal blocks (`false`, point-implicit).
    pub lines: bool,
    /// Linearise the ghost-cell states (BCs) in the implicit operator.
    pub implicit_bc: bool,
    /// Defect-correction passes against the unfactored operator (1 = plain approximate factorization).
    pub sub_iters: usize,
    /// Dissipation matrix of the implicit operator: Roe-type `|A|` of the face (`true`) or the scalar Rusanov `lambda I` (`false`).
    pub roe: bool,
    /// Entropy-fix level (fraction of `c`) of the Roe-type dissipation matrix.
    pub eps_fix: f64,
    /// Weiss-Smith low-Mach preconditioning of the implicit left-hand side (2D: `LowMach` of this file, 3D: [`crate::lowmach5`], both paths of [`LineSolve3`]; the reference velocity is bounded below by
    /// `LOW_MACH_UREF_EPS c_inf` and `LOW_MACH_UINF_CUTOFF |u_inf|`). The right-hand side and so the converged state are unchanged; it only speeds up
    /// the convergence of low-Mach runs. `false` is the plain scheme.
    pub low_mach_precond: bool,
    /// 3D only: how the three directional factors are solved ([`LineSolve3`]).
    pub line_solve: LineSolve3,
    /// `Some(omega)`: the diagonally dominant approximate factorisation of the block lines (DDADI; `LineSolve3::Block` only) with the correction multiplied by `omega`. The unfactored
    /// operator is `A = D + O_i + O_j + O_k` with the full diagonal `D = P + dt sum_d Dg_d` (`Dg_d` the diagonal blocks of the directional Jacobians, `P` the identity without the
    /// preconditioning); the factors are `D + O_d` instead of `P + dt Dg_d + O_d`, and `D` stands between them where `P` stood. The product of the factors is `A` plus the cross terms
    /// `O_i D^-1 O_j ...`, whose size the full diagonal bounds as the CFL grows; with the plain factors it grows with the CFL until the defect-correction passes diverge (the defect
    /// `|b - A du| / |b|` of the factored solve on the Draft example sector: 0.03 at an effective CFL of 2.7, 1.1-1.4 at 18-20 for the plain factors, 0.2-0.8 from 2.7 to 50 with DDADI,
    /// every pass then reducing it by about 0.7). The factorisation error of the plain factors also damps a period-2 mode of the accurate solve (the first-order left-hand side against the
    /// limited second-order residual), which `omega < 1` has to replace: `omega = 1` stalls on a limit cycle at CFL >= 6 on the viscous block of the tests, `omega = 0.5` converges to
    /// the state of the plain scheme from CFL 3 to 300. The converged state is that of the plain scheme, only the path changes. CPU (f64) measurements in DESIGN3D. The 3D GPU block scheme
    /// (`gpu/implicit3.rs`) implements it; the 2D scheme and the diagonalised line solve ignore this field.
    pub ddadi: Option<f64>,
    /// `delta`: the face dissipation of the implicit left-hand side (inviscid and viscous, every factor and the defect operator) is multiplied by `1 + delta`. The right-hand
    /// side and so the converged state are unchanged. Meant for the DDADI solve at `omega = 1`: the odd-even mode of an accurate first-order solve against the limited
    /// MUSCL residual has the amplification `1 - (1 + a - b) 2 sigma / (1 + 2 sigma) / (1 + delta)` at large CFL (`1 + a - b` up to 2.3 for van Albada, 2 for minmod), so
    /// `delta` 0.3-0.5 keeps it above -1 while the smooth modes change by `O(delta theta)`. 0 is the plain scheme.
    pub lhs_dis_boost: f64,
}

/// Lower bound of the reference velocity of the low-Mach preconditioning in units of the free-stream sound speed. Measured on the example
/// drag table (10 viscous Mach points, coarse grid, RTX 3090, df32): 0.02, 0.05 and 0.1 gave identical iteration counts and `C_D` at Mach 0.10,
/// 0.50 and 0.90; 0.1 is the value of the table runs below.
pub const LOW_MACH_UREF_EPS: f64 = 0.1;

/// Turkel's cut-off of the low-Mach preconditioning: the reference velocity is also bounded below by this multiple of the free-stream speed, so
/// `P = I` in the free stream of a supersonic run (and nearly so in its far field). Free of cost; on the GPU the table runs were stable without it too.
pub const LOW_MACH_UINF_CUTOFF: f64 = 1.0;

/// How the directional factors of the 3D approximate factorisation ([`crate::implicit3`]) are solved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineSolve3 {
    /// Block-tridiagonal lines of 5x5 blocks in each direction (the scheme of [`crate::implicit3`]).
    Block,
    /// Diagonalised factors (Pulliam-Chaussee: per cell `T (I + dt delta Lambda) T^-1`, five scalar tridiagonal systems per line) as the approximate
    /// inverse of one defect-correction pass against the BLOCK operator `I - dt J_i - dt J_j - dt J_k`:
    /// `dU0 = D^-1 b`, `dU = dU0 + D^-1 (b - (I - dt J_i - dt J_j - dt J_k) dU0)`. `sub_iters` is not used (this is one pass). Measured on the
    /// 3D ANF case (level 0, f32, `cfl_max` 10, RTX 3090): a step 3151 -> 1935 us, the same iterations and `C_N` (within 0.01 %) as `Block` at
    /// Mach 0.5, 0.8, 2 and 3, 1.56x faster to the stop rule. A single diagonal pass without the correction settles at a `C_N` 0.25 % off at
    /// Mach 0.8 and is not offered.
    DiagonalCorrected,
}

impl ImplicitParams {
    pub const DEFAULT: ImplicitParams = ImplicitParams {
        cfl0: 1.0,
        cfl_max: 30.0,
        growth: 1.02,
        dt_mode: DtMode::Sum,
        lines: true,
        implicit_bc: true,
        sub_iters: 2,
        roe: true,
        eps_fix: 0.1,
        low_mach_precond: false,
        line_solve: LineSolve3::Block,
        ddadi: None,
        lhs_dis_boost: 0.0,
    };

    /// Parameters for viscous runs: the weaker-direction pseudo-time step with a lower CFL limit and one more defect-correction pass.
    pub const VISCOUS: ImplicitParams = ImplicitParams { dt_mode: DtMode::MinDirection, cfl_max: 10.0, sub_iters: 3, ..ImplicitParams::DEFAULT };
}
