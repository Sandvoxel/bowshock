//! The Menter SST k-omega model's constants, the free-stream values of its scalars, the Aupoix rough-wall state and the viscous-flow parameters
//! ([`Viscous`], [`Turbulence`], the Prandtl numbers) shared by every solver and engine: the dimension-independent part of the model, which the 3D
//! kernels of [`crate::turbulence3`] and [`crate::viscous3`] take their constants from.

/// Turbulence model of a viscous run: `Laminar` (`mu_t = 0`) or the Menter SST k-omega model (`turbulence.rs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turbulence {
    Laminar,
    Sst,
}

/// Viscous-flow parameters (non-dimensionalisation of `viscous.rs`: `rho_inf = c_inf = 1`, lengths in nose diameters).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viscous {
    /// Reynolds number `rho_inf c_inf d_nose / mu_inf` of the free stream.
    pub re_c: f64,
    /// Sutherland constant `110.4 K / T_inf[K]`.
    pub sutherland: f64,
    pub turbulence: Turbulence,
    /// First `i` index (column) of the no-slip surface: the wall upstream of it (e.g. the axis or the approach to a plate)
    /// stays a slip wall.
    pub wall_i_start: usize,
    /// Equivalent sand-grain roughness `ks` of the no-slip walls in nose diameters; `0.0` is a smooth wall. With `ks > 0` the SST model
    /// uses the Wilcox (1988) rough-wall `omega` at the wall and the Hellsten & Laine (1997) `F3` limiter correction, see `turbulence.rs`.
    pub roughness: f64,
}

/// Laminar Prandtl number.
pub const PRANDTL: f64 = 0.72;
/// Turbulent Prandtl number.
pub const PRANDTL_T: f64 = 0.9;

impl Viscous {
    /// Dynamic viscosity (Sutherland's law) at the static temperature `t` (free-stream temperature = 1).
    #[inline]
    pub fn mu(&self, t: f64) -> f64 {
        t.powf(1.5) * (1.0 + self.sutherland) / (t + self.sutherland) / self.re_c
    }

    /// Does the run use a rough wall (SST with `roughness > 0`)? Then `Solver::rough_wall` holds the Aupoix wall state of the wall cells.
    #[inline]
    pub fn rough(&self) -> bool {
        self.turbulence == Turbulence::Sst && self.roughness > 0.0
    }
}

/// Thermal condition of the no-slip walls of a 3D run (`Case3`, `RansOpts3`; the 2D solver has adiabatic walls only).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum WallThermal {
    /// Zero normal temperature gradient: the wall ghost mirrors `T` (the default; bit-identical to a build without the option).
    #[default]
    Adiabatic,
    /// Isothermal wall at `T_w / T_inf` (the free-stream static temperature is 1 in the solver's units).
    Isothermal(f64),
}

/// An isothermal wall resolved against the gas law: the wall temperature and the laminar viscosity `mu(T_w)` at it. Kernels take `Option<IsoWall>`, `None` for an adiabatic wall.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IsoWall {
    pub t_w: f64,
    pub mu_w: f64,
    pub mu_face: bool,
    pub law: bool,
    pub lam2: bool,
}

impl WallThermal {
    /// The isothermal wall of this setting for the gas `v` (`None`: adiabatic).
    pub fn iso(&self, v: &Viscous) -> Option<IsoWall> {
        match *self {
            WallThermal::Adiabatic => None,
            WallThermal::Isothermal(t_w) => {
                assert!(t_w.is_finite() && t_w > 0.0, "wall temperature must be positive: {t_w}");
                Some(IsoWall {
                    t_w,
                    mu_w: v.mu(t_w),
                    mu_face: std::env::var_os("ISO_NO_MU").is_none(),
                    law: std::env::var_os("ISO_NO_LAW").is_none(),
                    lam2: std::env::var_os("ISO_NO_LAM2").is_none(),
                })
            }
        }
    }
}

/// Free-stream specific turbulence kinetic energy.
pub const K_INF: f64 = 9e-9;

/// Free-stream specific dissipation rate `1e-6 re_c`.
pub fn omega_inf(re_c: f64) -> f64 {
    1e-6 * re_c
}

/// Model constants of the SST model (see the module documentation); public so that GPU ports can take them from here.
pub const A1: f64 = 0.31;
pub const BETA_STAR: f64 = 0.09;
pub const SIGMA_K1: f64 = 0.85;
pub const SIGMA_K2: f64 = 1.0;
pub const SIGMA_W1: f64 = 0.5;
pub const SIGMA_W2: f64 = 0.856;
pub const BETA1: f64 = 0.075;
pub const BETA2: f64 = 0.0828;
pub const GAMMA1: f64 = 5.0 / 9.0;
pub const GAMMA2: f64 = 0.44;

/// Fixed-point iterations of `aupoix_wall` (and its GPU twin). The map `u_tau^2 -> (nu + nu_t,w) U / d` contracts by at most about 0.65 per iteration, from
/// a start that is up to a factor `1 + nu_t,w / nu` (10 and more at a leading edge) below the fixed point: 4 iterations leave that error unresolved and the
/// plate of G2r did not converge (density residual stuck at 0.5-1), 5 or more converge; 16 leave under 1e-3.
pub const AUPOIX_ITERATIONS: usize = 16;

/// Upper bound of `nu_t,w / nu`: the cap `1e5 / re_c` of the eddy viscosity of the cells (`eddy_viscosity`) in units of the free-stream `nu`. Where the
/// smooth `omega_w` bound is active (`d` large against the roughness) `nu_t,w` grows with `u_tau^2` and the fixed point has no solution without it.
pub const NU_T_WALL_MAX: f64 = 1e5;

/// Number of wall cells over which the roughness is phased in from the wall start (`Viscous::wall_i_start`): the roughness of the first wall cells is
/// `ks rough_ramp(i)`. A roughness wall state that starts as a step feeds turbulence (`k_w` with the low rough `omega_w`) into the slip cells ahead of the
/// wall start, where it spreads up their column (`mu_t / mu` 15-45 against 3 smooth) and the column oscillates with period 2 at transonic speed (M 0.9-1.3,
/// ks / d 6e-4 to 1.2e-3: the stall at 80 000 iterations); a ramp of 2 and 3 cells still oscillates, 4 and 16 do not.
pub const ROUGH_RAMP_CELLS: usize = 8;

/// Fraction of the roughness `ks` that wall cell `i` carries: `min(1, (i + 1 - wall_i_start) / ROUGH_RAMP_CELLS)`.
#[inline]
pub fn rough_ramp(i: usize, wall_i_start: usize) -> f64 {
    ((i + 1).saturating_sub(wall_i_start) as f64 / ROUGH_RAMP_CELLS as f64).min(1.0)
}

/// Aupoix (2015) "Colebrook" rough-wall state of a no-slip wall cell with kinematic viscosity `nu`, speed `speed`, wall distance `d`
/// and equivalent sand-grain roughness `ks > 0`: `[k_w, omega_w, nu_t,w]` with `nu_t,w = k_w / omega_w`.
///
/// ```text
///   ks+      = u_tau ks / nu                                                    (floored at 1e-30)
///   k_w+     = max(0, tanh( (log10(ks+ / 30) + 1 - tanh(ks+ / 125)) tanh(ks+ / 125) ) / sqrt(beta*))
///   omega_w+ = (300 / ks+^2) / tanh(15 / (4 ks+)) + (191 / ks+) (1 - exp(-ks+ / 250))
///   k_w = k_w+ u_tau^2,   omega_w = min(omega_w+ u_tau^2 / nu, 60 nu / (beta1 d^2))
///   u_tau^2 = (nu + nu_t,w) speed / d      (the discrete wall shear (mu + mu_t,w) U / d), `AUPOIX_ITERATIONS` fixed-point iterations from nu speed / d
/// ```
///
/// `omega_w+ u_tau^2 / nu` is evaluated as `300 nu / ks^2 / tanh(..) + 191 u_tau / ks (1 - exp(..))`, which stays finite at rest.
pub fn aupoix_wall(nu: f64, speed: f64, d: f64, ks: f64) -> [f64; 3] {
    let smooth = 60.0 * nu / (BETA1 * d * d);
    let at = |u2: f64| -> [f64; 3] {
        let u_tau = u2.sqrt();
        let ks_plus = (u_tau * ks / nu).max(1e-30);
        let t = (ks_plus / 125.0).tanh();
        let k_plus = (((ks_plus / 30.0).log10() + 1.0 - t) * t).tanh().max(0.0) / BETA_STAR.sqrt();
        let om_rough = 300.0 * nu / (ks * ks) / (15.0 / (4.0 * ks_plus)).tanh() + 191.0 * u_tau / ks * (1.0 - (-ks_plus / 250.0).exp());
        let (k_w, om_w) = (k_plus * u2, om_rough.min(smooth));
        [k_w, om_w, (k_w / om_w).min(NU_T_WALL_MAX * nu)]
    };
    let mut u2 = nu * speed / d;
    for _ in 0..AUPOIX_ITERATIONS {
        u2 = (nu + at(u2)[2]) * speed / d;
    }
    at(u2)
}
