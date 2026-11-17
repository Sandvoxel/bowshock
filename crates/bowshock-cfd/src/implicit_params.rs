//! Parameters of the line-implicit pseudo-time stepping ([`ImplicitParams`], [`DtMode`], [`LineSolve3`]) shared by the CPU solver ([`crate::implicit3`]), the
//! low-Mach preconditioning ([`crate::lowmach5`]), the GPU ports and the engine: what is not part of either solver's kernels.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtMode {
    /// `dt = CFL A / (L_i + L_j)` (the explicit definition).
    Sum,
    /// `dt = CFL A / (2 min(L_i, L_j))`: sized by the weaker direction.
    MinDirection,
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
    };

    /// Parameters for viscous runs: the weaker-direction pseudo-time step with a lower CFL limit and one more defect-correction pass.
    pub const VISCOUS: ImplicitParams = ImplicitParams { dt_mode: DtMode::MinDirection, cfl_max: 10.0, sub_iters: 3, ..ImplicitParams::DEFAULT };
}
