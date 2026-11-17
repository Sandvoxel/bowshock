//! Menter SST k-omega turbulence model (2003 version) in three dimensions, written as a list of scalar transport equations, with the automatic all-y+ wall
//! treatment: the 3D counterpart of [`crate::turbulence`] (same constants, limiters, free-stream values and clipping), built on the conventions of
//! [`crate::viscous3`] (read its module header: ghost in the face plane, `SymSides`, per-cell `wall` state, solid mask).
//!
//! Units as in 2D: `rho_inf = c_inf = 1`, lengths in nose diameters, `nu = mu / rho`. The update is a sequence of data-parallel kernels of plain, explicit inputs
//! ([`TurbIn`]: grid, solid mask, padded primitives `w`, the face-flux arrays for the mass flow, the gradients and viscosity of `viscous3::cell_pass`, the wall
//! distance of [`crate::walldist3`], the wall state, `dt`, the scalars): it never sees a `Solver3` and keeps no state. Per equation `e` of the list (`N`
//! scalars per cell; SST has `N = 2`, `[k, omega]`):
//!
//! ```text
//!   S^2 = 2 S_ij S_ij - 2/3 div^2   (full 3D strain from the cell gradient; the 2D formula with the hoop terms inside the gradient)
//!   X   = 2 rho sigma_w2 (grad k . grad omega) / omega,  CD = max(X, 1e-20)
//!   F1  = tanh(a^4),  a = min( max( sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) ), 4 rho sigma_w2 k / (CD d^2) );  F2 = tanh(b^2),  b = max( 2 sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) )
//!   mu_t = min( rho a1 k / max(a1 omega, S F2), 1e5 / re_c );   P = min( mu_t S^2, 10 beta* rho k omega )
//!   rho dk/dt     = (-conv_k + diff_k) / V + P - beta* rho omega k
//!   rho domega/dt = (-conv_w + diff_w) / V + gamma rho P / max(mu_t, 1e-30) - beta rho omega^2 + (1 - F1) X
//!   conv  = sum over the six faces of (outward mass flow) (phi_upwind - phi_c), inflow faces only; the mass flow of a face is `F[0] A` of the inviscid face flux
//!   diff  = sum over the six faces of the outward (mu_f + sigma_f mu_t,f) (g_f . n) A,  g_f = gbar + ((phi_R - phi_L) / d - gbar.e) e  (as the velocity of viscous3,
//!           ghost in the plane of the face, reflected scalar gradient `R g` across a symmetry plane), sigma_f the mean of the blended sigma of the two sides
//! ```
//!
//! Linear systems (the 2D ones with a third factor): with `diag0 = rho V / dt + rho V D` (`D_k = beta* omega`, `D_w = 2 beta omega`), `M-` / `M+` the mass flow
//! in the positive direction through the lower / upper face of a line direction and `G = (mu_f + sigma_f mu_t,f) A / d`, per direction
//! `lo = -(max(M-, 0) + G-)`, `up = -(max(-M+, 0) + G+)`, `dd = max(M-, 0) + max(-M+, 0) + G- + G+`, and
//!
//! ```text
//!   i lines: (diag0 + dd_i) x_c + lo_i x_{c-1} + up_i x_{c+1} = V R_c                              R_c = rho dphi/dt
//!   j lines: (1 + dd_j / diag0) y_c + (lo_j / diag0) y_{c-1} + (up_j / diag0) y_{c+1} = x_c        k lines: the same with the k coefficients, right-hand side y
//!   phi_new = max(phi + dphi, 0.1 phi), then k >= 1e-3 k_inf, omega >= 1e-3 omega_inf;  then mu_t is recomputed
//! ```
//!
//! A neighbour that is a ghost, a solid cell or across a zero-area face drops its off-diagonal (the diagonal part stays); solid rows are the identity, so a line
//! through a fin or a body solves as independent segments. The scalar tridiagonal solve is a plain Thomas algorithm (the one of `implicit3` is a private 5x5
//! block solver).
//!
//! # Ghost rules (as the 2D module)
//!
//! A domain side takes [`ScalarBc`]: `Inf` the free-stream values `(k_inf, omega_inf)` (far field, outflow), `Cell` the cell values (symmetry planes, axis: with the
//! reflected gradient the normal derivative is 0). A no-slip face takes the wall state `[k_w, omega_w, mu_t,w]` of its cell: the ghost is `2 phi_w - phi_c`, so the
//! face value is `k_w`, `omega_w`; the face eddy viscosity is `mu_t,w` (via `viscous3::face_mu3`, so the mean-flow wall flux and the scalar diffusion use one).
//!
//! # Differences from `turbulence.rs` (read this before integrating or porting)
//!
//! 1. **Ghosts that follow their cell add nothing to the diagonal** of the line system (`ScalarBc::Cell`: symmetry planes, axis, zero gradient). The 2D module keeps the
//!    diagonal of every dropped neighbour; for a wedge `k` face the conductance is `mu A_k / (r dtheta)`, an `O(1 / dtheta^2)` damping of every update, so the exact
//!    identity linearisation (ghost = cell: the off-diagonal moves onto the diagonal and cancels it) is used there. Frozen ghosts (`Inf`, walls) keep the 2D diagonal.
//! 2. **`wall` is always populated** for every wall cell (the automatic treatment gives `mu_t,w` and `omega_w`, `k_w` at any `y+`), not only for rough walls.
//! 3. **`S^2`** is `2 S_ij S_ij - 2/3 div^2` of the full 3D cell gradient; the 2D hoop terms are inside the Cartesian gradient.
//! 4. **Scalar list**: `N` scalars per cell behind [`TurbModel`]; SST is [`Sst`], `N = 2`. A transition scalar is a model with `N = 3` that delegates `k`, `omega` (the
//!    `a_third_scalar_is_an_addition` test does exactly that with a passive scalar and gets bit-identical `k`, `omega`, `mu_t`).
//! 5. **Three factors** (i, j, k) with a scalar Thomas solve (`implicit3`'s tridiagonal solver is a private 5x5 block solver; the one of the 2D module is 4x4).
//! 6. **Per-cell roughness** `ks` array and per-cell wall state (a cell with two wall faces has one state, nearest wall); `rough_ramp` is the caller's.
//! 7. **Conventions shared with `viscous3`**: the scalar gradient across a symmetry plane is reflected (`R g`), the ghost is in the plane of the face (`FaceDirs3`: `e`, `d`,
//!    no-slip and symmetry flags), the mass flow of a face is `F[0] A` of the solver's inviscid flux arrays (zero through walls and symmetry planes).
//!
//! # Wall treatment (DESIGN3D section B): automatic, all y+
//!
//! For every fluid cell with a no-slip face ([`wall_state_pass`]), from its speed `U = |u|`, viscosity, density, the distance `d` of its centroid to the nearest wall
//! face (one state for every wall face of the cell: a junction or tip-corner cell takes the nearest wall's distance and roughness for all its wall faces) and the
//! roughness `ks` of the cell (an array: body and fins may differ; `0` smooth; the roughness ramp along a body is the caller's, put it into the array):
//!
//! ```text
//!   smooth (ks = 0):  u_tau  = root of  u U+(d u / nu) = U,   U+ = law_uplus: Reichardt's law of the wall (viscous sublayer, buffer layer and log layer in one function)
//!                              times (1 + 0.04 exp(-(ln(y+ / 6) / 0.5)^2)) (the buffer-layer correction towards the SST model's own profile), by 8 Newton iterations
//!                     tau_w = rho u_tau^2,   y+ = d u_tau / nu
//!                     mu_t,w = max(rho u_tau^2 d / U - mu, 0)      (so the wall flux (mu + mu_t,w) U / d IS tau_w: viscous3's momentum flux through the wall face)
//!                     omega_w = sqrt(omega_vis^2 + omega_log^2),  omega_vis = 6 nu / (beta1 d^2) (1 + 9 / (1 + (y+ / 4)^4)),  omega_log = u_tau / (sqrt(beta*) kappa d)
//!                     k_w = u_tau^2 / sqrt(beta*) y+^4 / (y+^4 + 12^4)
//!                     S^2 of the cell = ((u_tau^2 / nu) dU+/dy+)^2  instead of its Green-Gauss gradient (TurbIn::law_strain)
//!                     k of the cell <= (1 - w) k + w k_w  with w = (y+/4)^8 / (1 + (y+/4)^8)   (Sst::wall_cell)
//!   rough (ks > 0):   [k_w, omega_w, nu_t,w] = turbulence::aupoix_wall(nu, U, d, ks) verbatim (replaces the law of the wall),  mu_t,w = rho nu_t,w,  u_tau^2 = (nu + nu_t,w) U / d
//! ```
//!
//! with `kappa = 0.41`. Where the DESIGN3D recipe is not followed literally, and why:
//!
//! * `u_tau` is the inverse of Reichardt's law, not the 4th-power blend `(u_vis^4 + u_log^4)^(1/4)` of Esch & Menter (kept as [`u_tau_blend`]): against Reichardt's law
//!   the blend's wall shear `u_tau^2` is `1.2 %` high at `y+ = 1`, `4.2 %` at 5, `5.4 %` at 15, `6.9 %` at 30, `2.7 %` at 100 (the table of
//!   `wall_state_follows_the_sublayer_and_the_log_layer`), which would eat the 5 % friction gate of G-V2 at the fin `y+` of the Coarse tier; the inverse of the law is exact
//!   at every `y+` (by construction) and equals the viscous `nu U / d` of the sublayer to `0.8 %` at `y+ = 1` (where `mu_t,w` is clamped to 0, so the 2D wall flux is
//!   reproduced exactly).
//! * `omega_vis` carries the factor `1 + 9 / (1 + (y+ / 4)^4)`: Menter's wall value `60 nu / (beta1 d^2) = 10 * 6 nu / (beta1 d^2)` of the 2D module for `y+ <~ 2` (0.4 % low
//!   at `y+ = 1`), rolled off to the exact sublayer `6 nu / (beta1 d^2)` by `y+ ~ 15`. With the factor 10 kept, `omega_w` would exceed the log-layer `omega_log` by 3.4 at
//!   `y+ = 30`.
//! * `k_w`, the face value of `k` (the ghost is `2 k_w - k_c`), is 0 in the sublayer like the 2D wall and the log-layer `k = u_tau^2 / sqrt(beta*)` at `y+ >= 30`, so the first cell
//!   is not drained by a `k = 0` wall face where `k` is not small.
//! * **The strain of a wall cell comes from the law of the wall** ([`strain_law`]), not from the cell's gradient: the Green-Gauss gradient of a first cell is its velocity over
//!   its height, `1 / (kappa y+)` times the log-layer value, so the production `mu_t S^2` was several `10` times too high and `k+` reached `134` in a first cell of `y+ = 40`
//!   (`3.3` in the log layer). Equal to the gradient in the sublayer (the first cell of the `y+ = 0.36` plate is unchanged: `cf` identical to 5 digits).
//! * **`k` of a wall cell is capped** at the law-of-the-wall value from `y+ ~ 3` on ([`Sst::wall_cell`]): a first cell of `y+ = 6` still reached `k+ = 31` (`4.5` at most in a
//!   resolved layer). `omega` solves its equation (a pin of `k` and `omega` together to the law values blew up at the leading edge of the `y+ = 1` plate).
//! * **The law of the wall is Reichardt's with a `4 %` Gaussian correction at `y+ ~ 6`** ([`law_uplus`]): the profile of the resolved SST model lies `2 %` above Reichardt's there,
//!   which put the wall shear of the `y+ = 5` first-cell run `6.6 %` above that of the resolved run (`4.5 %` with the correction).
//!
//! Plate results (`plate3`, `Re_x = 4.85e6`, resolved `y+ = 0.51`: `cf = 2.6786e-3`): first-cell `y+ = 1.0 / 2.0 / 3.0 / 5.2 / 8.1 / 15.2 / 30.4` gives `-1.9 / -2.6 / +0.1 / +4.5 /
//! +0.9 / -0.7 / -0.1 %`.
//!
//! Numbers (`wall_state_follows_the_sublayer_and_the_log_layer`, wall values against the 2D ones `60 nu / (beta1 d^2)`): for `y+ <= 1` `mu_t,w = 0`, `k_w < 1e-3 u_tau^2`,
//! `omega_w` within 0.4 % of the 2D value, the wall shear within 1 % of the viscous one; for `y+ >= 30` the wall shear is within 4 % of the log law (Reichardt's constant
//! differs from `E = 9.793` by 2 % in `U+`), `omega_w` within 6 % of `omega_log` and `k_w` of `u_tau^2 / sqrt(beta*)` within 7 %.

use crate::gas3::{Cons3, Prim3};
use crate::grid3::Grid3;
use crate::solver3::pidx3;
use crate::sst::{aupoix_wall, A1, BETA1, BETA2, BETA_STAR, GAMMA1, GAMMA2, NU_T_WALL_MAX, SIGMA_K1, SIGMA_K2, SIGMA_W1, SIGMA_W2};
use crate::viscous3::{face_cells, face_mu_cells, face_of, face_shape, map_faces, FaceCells, FaceDirs3, Grad3, SideKind};
use rayon::prelude::*;

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// wall treatment

/// Von Karman constant of the law of the wall (`0.41`, the value the SST constants imply: `kappa^2 = sqrt(beta*) (beta1 / beta* - gamma1) / sigma_w1 = 0.1667`).
pub const KAPPA: f64 = 0.41;
/// Log-law constant `E = exp(kappa C)` of the reference blend [`u_tau_blend`] (`U+ = ln(E y+) / kappa`, `C = 5.56`).
pub const LOG_E: f64 = 9.793;
/// Menter's wall-value factor of `omega` (`60 nu / (beta1 d^2) = 10 * 6 nu / (beta1 d^2)`), kept for `y+ <~ 2` and rolled off to 1 by `y+ ~ 15`.
pub const OMEGA_BOOST: f64 = 10.0;
/// `y+` at which the boost of `omega_vis` has lost half of its excess.
pub const Y_BOOST: f64 = 4.0;
/// `y+` at which the wall value of `k` (and the cap of `k` in a wall-adjacent cell, [`Sst::wall_cell`]) reaches half of the log-layer value `u_tau^2 / sqrt(beta*)` (a
/// fourth-power switch: `k+` is `1.6e-4` at `y+ = 1`, `0.07` at 5, `0.5` at 8, `2.2` at 15, `3.2` at 30; the SST profile of a resolved boundary layer has `0.5` at 5.7, `1.5`
/// at 9, `2.4` at 17, `2.9` at 29, `3.2` at 100).
pub const Y_K: f64 = 12.0;
/// `y+` at which the cap of `k` in a wall-adjacent cell has half weight (`weight = x / (1 + x)`, `x = (y+ / Y_PIN)^8`): `0.004` at `y+ = 2`, `0.1` at 3, `0.89` at 5.2, `1` from
/// 8: an eighth power, so a wall-resolved cell (`y+ < 2.5`) solves its equations untouched (a fourth power let the cap destabilise the leading edge of the plate at `y+ = 1`).
pub const Y_PIN: f64 = 4.0;
/// Newton iterations of the inversion of the law of the wall (from `u_vis`; the error after 8 is below `1e-12` for `y+ <= 1e4`).
pub const WALL_ITERATIONS: usize = 8;

/// What the wall treatment of one cell yields: the friction velocity, the wall values of `k` and `omega` and the wall eddy viscosity `mu_t,w`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallState {
    pub u_tau: f64,
    pub k_w: f64,
    pub omega_w: f64,
    pub mu_t_w: f64,
}

/// Reichardt's law of the wall `U+(y+) = ln(1 + kappa y+) / kappa + 7.8 (1 - exp(-y+/11) - (y+/11) exp(-y+/3))`: the viscous sublayer `U+ = y+`, the buffer
/// layer and the log layer (asymptote `ln(kappa y+) / kappa + 7.8`) in one smooth monotone function.
#[inline]
pub fn reichardt_uplus(yp: f64) -> f64 {
    (1.0 + KAPPA * yp).ln() / KAPPA + 7.8 * (1.0 - (-yp / 11.0).exp() - yp / 11.0 * (-yp / 3.0).exp())
}

/// `d U+ / d y+` of [`reichardt_uplus`]: 1 in the viscous sublayer, `1 / (kappa y+)` in the log layer.
#[inline]
pub fn reichardt_duplus(yp: f64) -> f64 {
    1.0 / (1.0 + KAPPA * yp) + 7.8 * ((-yp / 11.0).exp() / 11.0 - (-yp / 3.0).exp() / 11.0 + yp / 33.0 * (-yp / 3.0).exp())
}

/// Amplitude, centre (`y+`) and width (in `ln y+`) of the Gaussian correction of Reichardt's law. The velocity profile of the resolved SST model (`y+ = 0.36` plate,
/// `x = 0.97`) lies `2 %` above Reichardt's between `y+ = 5` and `10` (`5.62 / 5.51` at 5.7, `6.92 / 6.75` at 7.4, `8.11 / 7.96` at 9.3) and `3 - 4 %` below it between 20 and 60
/// (`11.67 / 12.01` at 20, `12.96 / 13.46` at 29, `14.08 / 14.59` at 40). The law the wall treatment inverts should be the model's own where a wall cell does not resolve the
/// layer: against Reichardt's the wall shear of a first cell at `y+ = 5.2` was `6.6 %` above that of the resolved run. Only the upper bump is corrected, with `4 %` (twice the
/// excess of the resolved profile: the wall cell is first-order accurate where the profile bends) so that the first-cell `y+ = 5` run is within the `5 %` of the G-V2 sweep (`4.5 %`
/// with it, `6.6 %` without); the grids of `y+ >= 8` are within `1 %` of the resolved run without any lower correction.
pub const LAW_BUMP: f64 = 0.04;
pub const LAW_BUMP_Y: f64 = 6.0;
pub const LAW_BUMP_W: f64 = 0.5;

/// The Gaussian correction `b(y+)` of the law of the wall and its derivative `db / dy+`.
#[inline]
fn law_bump(yp: f64) -> (f64, f64) {
    if yp < 1e-3 {
        return (0.0, 0.0);
    }
    let t = (yp / LAW_BUMP_Y).ln() / LAW_BUMP_W;
    let b = LAW_BUMP * (-t * t).exp();
    (b, b * (-2.0 * t / (LAW_BUMP_W * yp)))
}

/// The law of the wall of the wall treatment, `U+(y+) = U+_Reichardt (1 + b)`: viscous sublayer, buffer layer and log layer of the SST model in one smooth monotone function.
#[inline]
pub fn law_uplus(yp: f64) -> f64 {
    reichardt_uplus(yp) * (1.0 + law_bump(yp).0)
}

/// `d U+ / d y+` of [`law_uplus`].
#[inline]
pub fn law_duplus(yp: f64) -> f64 {
    let (b, db) = law_bump(yp);
    reichardt_duplus(yp) * (1.0 + b) + reichardt_uplus(yp) * db
}

/// The strain rate `dU/dy = (u_tau^2 / nu) dU+/dy+` the law of the wall has at the centroid of a wall cell (distance `d`, speed `speed`, kinematic viscosity `nu`): the
/// wall shear rate `U / d` in the viscous sublayer, `u_tau / (kappa d)` in the log layer. The mean-flow gradient of the cell is neither there: its Green-Gauss
/// value is the first-cell velocity over the cell height (`U / d`), `1 / (kappa y+)` times too large in the log layer, which feeds the production of `k`
/// `mu_t S^2` of the cell with `S^2` several `10` times too high (measured: `k+ = 134` in a first cell of `y+ = 40`, `3.3` expected).
pub fn strain_law(nu: f64, speed: f64, d: f64) -> f64 {
    let u = u_tau_law(nu, speed, d);
    u * u / nu * law_duplus(d * u / nu)
}

/// The friction velocity of the law of the wall ([`law_uplus`]) at speed `speed`, distance `d`, kinematic viscosity `nu`: the root of `u U+(d u / nu) = U` by Newton iteration from
/// the viscous value `u_vis = sqrt(nu U / d)` (`F(u) = u U+(y+) - U` is increasing and convex, so the iteration converges monotonically after the first step;
/// a step is limited to halving `u`). Zero at rest.
pub fn u_tau_law(nu: f64, speed: f64, d: f64) -> f64 {
    if speed <= 0.0 {
        return 0.0;
    }
    let mut u = (nu * speed / d).sqrt();
    for _ in 0..WALL_ITERATIONS {
        let yp = d * u / nu;
        let (up, dup) = (law_uplus(yp), law_duplus(yp));
        u = (u - (u * up - speed) / (up + yp * dup)).max(0.5 * u);
    }
    u
}

/// The blend `u_tau = (u_vis^4 + u_log^4)^(1/4)` of Esch & Menter (2003), `u_vis = sqrt(nu U / d)`, `u_log = kappa U / max(ln(E d u_log / nu), 1)` (5 fixed-point
/// iterations): the reference the law of the wall [`u_tau_law`] replaced. Against Reichardt's law its wall shear `u_tau^2` is `1.2 %` high at `y+ = 1`,
/// `4.2 %` at 5, `5.4 %` at 15, `6.9 %` at 30, `2.7 %` at 100 (the `wall_state_table` test prints them), which would break a `5 %` friction gate in the buffer layer.
pub fn u_tau_blend(nu: f64, speed: f64, d: f64) -> f64 {
    let u_vis = (nu * speed / d).sqrt();
    let mut u = u_vis;
    for _ in 0..5 {
        u = KAPPA * speed / (LOG_E * d * u / nu).ln().max(1.0);
    }
    (u_vis.powi(4) + u.powi(4)).sqrt().sqrt()
}

/// Smooth-wall state of the automatic treatment (see the module documentation) for density `rho`, kinematic viscosity `nu`, speed `speed` of the cell centroid
/// and its distance `d` to the wall.
pub fn wall_state_smooth(rho: f64, nu: f64, speed: f64, d: f64) -> WallState {
    let u_tau = u_tau_law(nu, speed, d);
    let yp = d * u_tau / nu;
    let boost = 1.0 + (OMEGA_BOOST - 1.0) / (1.0 + (yp / Y_BOOST).powi(4));
    let omega_vis = 6.0 * nu / (BETA1 * d * d) * boost;
    let omega_log = u_tau / (BETA_STAR.sqrt() * KAPPA * d);
    let k_w = k_wall(u_tau, yp);
    let nu_t_w = if speed > 0.0 { (u_tau * u_tau * d / speed - nu).clamp(0.0, NU_T_WALL_MAX * nu) } else { 0.0 };
    WallState { u_tau, k_w, omega_w: omega_vis.hypot(omega_log), mu_t_w: rho * nu_t_w }
}

/// The wall value of `k` and the cap of `k` in a wall-adjacent cell: `u_tau^2 / sqrt(beta*) y+^4 / (y+^4 + Y_K^4)` (`0` in the viscous sublayer, the log-layer `k` above `y+ ~ 30`).
#[inline]
pub fn k_wall(u_tau: f64, yp: f64) -> f64 {
    let y4 = yp.powi(4);
    u_tau * u_tau / BETA_STAR.sqrt() * y4 / (y4 + Y_K.powi(4))
}

/// Wall state of a cell: the automatic smooth treatment for `ks == 0`, the Aupoix (2015) rough-wall state ([`aupoix_wall`], verbatim) for `ks > 0`, whose
/// fixed point also gives the friction velocity `u_tau^2 = (nu + nu_t,w) U / d`.
pub fn wall_state(rho: f64, nu: f64, speed: f64, d: f64, ks: f64) -> WallState {
    if ks > 0.0 {
        let [k_w, omega_w, nu_t_w] = aupoix_wall(nu, speed, d, ks);
        WallState { u_tau: ((nu + nu_t_w) * speed / d).sqrt(), k_w, omega_w, mu_t_w: rho * nu_t_w }
    } else {
        wall_state_smooth(rho, nu, speed, d)
    }
}

/// The wall state `[k_w, omega_w, mu_t,w]` of every fluid cell with a no-slip face (zero elsewhere), from the primitives `w` (padded), the viscosity `mu`
/// (`viscous3::cell_pass`), the wall distance `dwall` (`walldist3`) and the roughness per cell `ks` (empty: all smooth); this is the `wall` array of
/// `viscous3::face_mu3` and of [`update`]. Needed after every change of the mean flow, before the viscous face pass. The speed of the wall treatment is the one
/// tangential to the wall (`|u - (u.n) n|`, `n` = `FaceDir3::e` of a no-slip face; the largest over the cell's no-slip faces): the cells in front of the base plane
/// of a body see the recirculation hit the wall, a normal velocity that is no shear (with the total speed the base corner cells carried `tau` 1000 times a
/// log-law value at `y+ ~ 1600`).
///
/// **Wall classes** (`law_i_walls`): the automatic all-`y+` treatment is that of the streamwise walls, the no-slip faces of the `j` and `k` families (a body surface, the
/// sides and tip of a fin). A cell whose only no-slip faces are `i` faces (the base plane of a body, the blunt leading and trailing edge of a fin) gets the plain wall state
/// of a resolved wall, the `y+ -> 0` limit: `k_w = 0`, `omega_w` Menter's `60 nu / (beta1 d^2)`, `mu_t,w = 0` (speed `0` in [`wall_state`]); the log law means nothing in a
/// recirculation or at a stagnation face (G-V3: with the law on the base wall the base drag is `-3.2 %` against the 2D solver, plain `+0.35 %`). `law_i_walls = true` gives the `i`
/// faces the law as well. A rough cell (`ks > 0`) keeps the Aupoix state either way.
#[allow(clippy::too_many_arguments)]
pub fn wall_state_pass(
    grid: &Grid3,
    solid: &[bool],
    dirs: &FaceDirs3,
    w: &[Prim3],
    mu: &[f64],
    dwall: &[f64],
    ks: &[f64],
    law_i_walls: bool,
    out: &mut [[f64; 3]],
) {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let plane = ni * nj;
    out.par_chunks_mut(plane).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let c = grid.cell(i, j, k);
                pl[j * ni + i] = [0.0; 3];
                if solid[c] {
                    continue;
                }
                let lo = (k * nj + j) * (ni + 1) + i;
                let jl = (k * (nj + 1) + j) * ni + i;
                let faces = [dirs.i[lo].noslip, dirs.i[lo + 1].noslip, dirs.j[jl].noslip, dirs.j[jl + ni].noslip, dirs.k[c].noslip, dirs.k[c + plane].noslip];
                if !faces.iter().any(|&f| f) {
                    continue;
                }
                let wc = &w[pidx3(ni, nj, nk, i as isize, j as isize, k as isize)];
                let nu = mu[c] / wc[0];
                let speed = wall_speed(grid, dirs, i, j, k, wc, law_i_walls).unwrap_or(0.0);
                let s = wall_state(wc[0], nu, speed, dwall[c], ks.get(c).copied().unwrap_or(0.0));
                pl[j * ni + i] = [s.k_w, s.omega_w, s.mu_t_w];
            }
        }
    });
}

/// The speed of the cell `(i, j, k)` tangential to its no-slip faces of the law-of-the-wall class, `|u - (u.n) n|` with `n = FaceDir3::e` of the face (the largest over those
/// faces): the speed the law of the wall takes. `None` for a cell without such a face: the no-slip faces of the `i` family are of the law class only with `law_i_walls`
/// (see [`wall_state_pass`]), so a cell that has only those is a plain wall cell. A normal velocity is no shear: the cells in front of the base plane of a body see
/// the recirculation hit the wall.
#[inline]
pub fn wall_speed(grid: &Grid3, dirs: &FaceDirs3, i: usize, j: usize, k: usize, w: &Prim3, law_i_walls: bool) -> Option<f64> {
    let (ni, nj) = (grid.ni, grid.nj);
    let lo = (k * nj + j) * (ni + 1) + i;
    let jl = (k * (nj + 1) + j) * ni + i;
    let c = grid.cell(i, j, k);
    let faces = [&dirs.i[lo], &dirs.i[lo + 1], &dirs.j[jl], &dirs.j[jl + ni], &dirs.k[c], &dirs.k[c + ni * nj]];
    let speed2 = w[1] * w[1] + w[2] * w[2] + w[3] * w[3];
    let first = if law_i_walls { 0 } else { 2 };
    faces[first..]
        .iter()
        .filter(|f| f.noslip)
        .map(|f| {
            let un = w[1] * f.e[0] + w[2] * f.e[1] + w[3] * f.e[2];
            (speed2 - un * un).max(0.0).sqrt()
        })
        .reduce(f64::max)
}

/// The wall shear vector `tau_w = rho u_tau^2 t` (force per unit area of the wall on the fluid's tangential motion, i.e. opposing it) of a cell next to a wall of
/// unit normal `n`: `t` the unit vector of the tangential velocity `u - (u.n) n` (zero at rest). This is the model value; the mean-flow equations carry exactly
/// it as the tangential part of the wall momentum flux, `(mu + mu_t,w) u_t / d` with `mu_t,w` of [`wall_state`] (`viscous3::wall_tractions` reads the flux).
pub fn wall_shear_vector(rho: f64, u_tau: f64, vel: [f64; 3], n: [f64; 3]) -> [f64; 3] {
    let un = vel[0] * n[0] + vel[1] * n[1] + vel[2] * n[2];
    let t = [vel[0] - un * n[0], vel[1] - un * n[1], vel[2] - un * n[2]];
    let l = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
    if l == 0.0 {
        return [0.0; 3];
    }
    let s = rho * u_tau * u_tau / l;
    [s * t[0], s * t[1], s * t[2]]
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// the model: a list of N scalar transport equations

/// Gradient `[d_x, d_y, d_z]` of each of the `N` scalars of a cell.
pub type SGrad<const N: usize> = [[f64; 3]; N];

/// Everything a model needs of one cell.
#[derive(Clone, Copy, Debug)]
pub struct CellIn<const N: usize> {
    pub rho: f64,
    pub nu: f64,
    /// Distance of the centroid to the nearest wall (`f64::INFINITY` without a wall).
    pub d: f64,
    pub phi: [f64; N],
    pub grad: SGrad<N>,
    /// `S^2` of the mean flow ([`strain2`]).
    pub s2: f64,
    /// Current eddy viscosity of the cell.
    pub mu_t: f64,
}

/// Blending function `F1` and the cross diffusion `X` of a cell (what the SST sources and the face `sigma` need; a model without blending returns `f1 = 1`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Blend {
    pub f1: f64,
    pub x: f64,
}

/// A turbulence model as a list of `N` scalar transport equations `rho dphi_e/dt + conv_e = diff_e + S_e - rho D_e phi_e`: free-stream values, the diffusion
/// coefficients (`sigma_e = F1 sigma_e1 + (1 - F1) sigma_e2`), sources and linearised destruction, the wall face values, the clipping and the eddy viscosity.
/// A transition scalar is another implementor of the same kernels with `N = 3`.
pub trait TurbModel<const N: usize>: Sync {
    /// Free-stream values (`Inf` ghosts, initial state).
    fn inf(&self) -> [f64; N];
    /// `[sigma_1, sigma_2]` of equation `e`.
    fn sigmas(&self, e: usize) -> [f64; 2];
    /// `F1` and `X` of a cell.
    fn blend(&self, c: &CellIn<N>) -> Blend;
    /// Sources `S_e` (per unit volume, `= rho dphi/dt` contributions) and the linearised destruction coefficients `D_e` (`dphi/dt = ... - D_e phi`).
    fn terms(&self, c: &CellIn<N>, b: &Blend) -> ([f64; N], [f64; N]);
    /// The face value of equation `e` at a no-slip wall from the wall state `[k_w, omega_w, mu_t,w]` (`None`: zero gradient, the ghost is the cell value).
    fn wall_value(&self, e: usize, wall: &[f64; 3]) -> Option<f64>;
    /// The new value of equation `e` in a smooth wall-adjacent cell of centroid `y+` (friction velocity `u_tau`, kinematic viscosity `nu`) from the value the cell's own
    /// equation gave (`solved`); the default keeps it. The wall-function regime: a cell of `y+ ~ 5` and more does not resolve the sublayer its equations describe
    /// (measured with the `k` equation alone: `k+ = 31` in a first cell of `y+ = 6`, `4.5` at most in a resolved layer).
    fn wall_cell(&self, _e: usize, solved: f64, _y_plus: f64, _u_tau: f64, _nu: f64) -> f64 {
        solved
    }
    /// Clip of the updated value of equation `e`.
    fn clip(&self, e: usize, old: f64, new: f64) -> f64;
    /// The eddy viscosity from the (new) scalars of a cell.
    fn eddy_viscosity(&self, c: &CellIn<N>) -> f64;
}

/// `S^2 = 2 S_ij S_ij - 2/3 div^2` of a cell from its mean-flow gradient (`viscous3::Grad3`): the 2D `strain2` with the hoop terms contained in the 3D gradient.
#[inline]
pub fn strain2(g: &Grad3) -> f64 {
    let div = g[0] + g[4] + g[8];
    let mut s = 0.0;
    for a in 0..3 {
        for b in 0..3 {
            let sab = 0.5 * (g[3 * a + b] + g[3 * b + a]);
            s += sab * sab;
        }
    }
    2.0 * s - 2.0 / 3.0 * div * div
}

/// The Menter SST-2003 model: `[k, omega]`.
#[derive(Clone, Copy, Debug)]
pub struct Sst {
    /// Reynolds number `rho_inf c_inf d / mu_inf` (free-stream `omega_inf = 1e-6 re_c`, the eddy-viscosity cap `1e5 / re_c`).
    pub re_c: f64,
}

impl Sst {
    /// Argument pieces `(sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))` of the blending functions.
    #[inline]
    fn args(nu: f64, k: f64, om: f64, d: f64) -> (f64, f64) {
        (k.sqrt() / (BETA_STAR * om * d), 500.0 * nu / (d * d * om))
    }
}

impl TurbModel<2> for Sst {
    fn inf(&self) -> [f64; 2] {
        [crate::sst::K_INF, crate::sst::omega_inf(self.re_c)]
    }

    fn sigmas(&self, e: usize) -> [f64; 2] {
        if e == 0 {
            [SIGMA_K1, SIGMA_K2]
        } else {
            [SIGMA_W1, SIGMA_W2]
        }
    }

    fn blend(&self, c: &CellIn<2>) -> Blend {
        let [k, om] = c.phi;
        let g = &c.grad;
        let x = 2.0 * c.rho * SIGMA_W2 * (g[0][0] * g[1][0] + g[0][1] * g[1][1] + g[0][2] * g[1][2]) / om;
        let cd = x.max(1e-20);
        let (t1, t2) = Sst::args(c.nu, k, om, c.d);
        let t3 = 4.0 * c.rho * SIGMA_W2 * k / (cd * c.d * c.d);
        let a = t1.max(t2).min(t3);
        Blend { f1: a.powi(4).tanh(), x }
    }

    fn terms(&self, c: &CellIn<2>, b: &Blend) -> ([f64; 2], [f64; 2]) {
        let [k, om] = c.phi;
        let prod = (c.mu_t * c.s2).min(10.0 * BETA_STAR * c.rho * k * om);
        let beta = b.f1 * BETA1 + (1.0 - b.f1) * BETA2;
        let gamma = b.f1 * GAMMA1 + (1.0 - b.f1) * GAMMA2;
        (
            [prod - BETA_STAR * c.rho * om * k, gamma * c.rho * prod / c.mu_t.max(1e-30) - beta * c.rho * om * om + (1.0 - b.f1) * b.x],
            [BETA_STAR * om, 2.0 * beta * om],
        )
    }

    fn wall_value(&self, e: usize, wall: &[f64; 3]) -> Option<f64> {
        Some(wall[e])
    }

    /// `k` of a smooth wall-adjacent cell is capped at the law-of-the-wall value [`k_wall`] with the weight `x / (1 + x)`, `x = (y+ / Y_PIN)^8`: `min(solved, (1 - w) solved +
    /// w k_wall)`. Without the cap a cell of `y+ = 6` carries `k+ = 31` (`4.5` at most in a resolved layer: the production `mu_t S^2` of the cell feeds `mu_t = a1 k / S`, and
    /// the sublayer diffusion that bounds the resolved layer is missing); a cap only, never a raise, so the cell is not driven by the law (a pin of `k` and `omega` together
    /// blew up at the leading edge of the `y+ = 1` plate). `omega` solves its equation.
    fn wall_cell(&self, e: usize, solved: f64, y_plus: f64, u_tau: f64, _nu: f64) -> f64 {
        if e != 0 {
            return solved;
        }
        let x = (y_plus / Y_PIN).powi(8);
        let w = x / (1.0 + x);
        ((1.0 - w) * solved + w * k_wall(u_tau, y_plus)).min(solved)
    }

    fn clip(&self, e: usize, old: f64, new: f64) -> f64 {
        let inf = self.inf();
        new.max(0.1 * old).max(1e-3 * inf[e])
    }

    fn eddy_viscosity(&self, c: &CellIn<2>) -> f64 {
        let [k, om] = c.phi;
        let (t1, t2) = Sst::args(c.nu, k, om, c.d);
        let b = (2.0 * t1).max(t2);
        let f2 = (b * b).tanh();
        let s = c.s2.max(0.0).sqrt();
        (c.rho * A1 * k / (A1 * om).max(s * f2)).min(1e5 / self.re_c)
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------
// the update

/// Ghost of a domain side for the scalars (`Inf`: the free-stream values; `Cell`: the cell values, with the reflected gradient across a symmetry plane).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScalarBc {
    Inf,
    Cell,
}

/// `[i = 0, i = ni, j = 0, j = nj, k = 0, k = nk]`, as `viscous3::SymSides`.
pub type ScalarSides = [ScalarBc; 6];

/// Read-only data of one update. Arrays per cell (`ni nj nk`, index `Grid3::cell`) unless stated; `w` is `Solver3`'s padded primitive array.
pub struct TurbIn<'a, const N: usize> {
    pub grid: &'a Grid3,
    pub solid: &'a [bool],
    pub w: &'a [Prim3],
    /// `viscous3::face_dirs` of the grid (`e`, `d`, no-slip and symmetry flags of every face).
    pub dirs: &'a FaceDirs3,
    pub bc: &'a ScalarSides,
    /// Viscosity, eddy viscosity and mean-flow gradient of `viscous3::cell_pass` / the previous update.
    pub mu: &'a [f64],
    pub mu_t: &'a [f64],
    pub grad: &'a [Grad3],
    /// `[k_w, omega_w, mu_t,w]` of the wall cells ([`wall_state_pass`]); full length (entries of cells without a no-slip face are not read).
    pub wall: &'a [[f64; 3]],
    /// Roughness per cell (empty: all smooth).
    pub ks: &'a [f64],
    /// A smooth wall cell takes its `S^2` from the law of the wall ([`strain_law`]) instead of its mean-flow gradient (a rough one always keeps the gradient). `false`
    /// is the 2D module's treatment (the wedge parity tests); the solver runs with `true`.
    pub law_strain: bool,
    /// The no-slip faces of the `i` family take the automatic wall treatment too (`false`: a cell with only such faces is a plain wall cell, see [`wall_state_pass`]).
    pub law_i_walls: bool,
    /// Wall distance of the cell centroids (`walldist3`; `f64::INFINITY` without a wall).
    pub dwall: &'a [f64],
    /// Inviscid face fluxes (layouts of `Grid3::{iface, jface, kface}`): `F[0] A` is the mass flow the scalars are convected with.
    pub fi: &'a [Cons3],
    pub fj: &'a [Cons3],
    pub fk: &'a [Cons3],
    /// Pseudo time step per cell.
    pub dt: &'a [f64],
    /// The scalars of every cell.
    pub turb: &'a [[f64; N]],
}

impl<const N: usize> TurbIn<'_, N> {
    #[inline]
    fn rho(&self, c: usize) -> f64 {
        let g = self.grid;
        self.w[pidx3(g.ni, g.nj, g.nk, (c % g.ni) as isize, ((c / g.ni) % g.nj) as isize, (c / (g.ni * g.nj)) as isize)][0]
    }

    #[inline]
    fn flux(&self, dir: usize) -> &[Cons3] {
        match dir {
            0 => self.fi,
            1 => self.fj,
            _ => self.fk,
        }
    }

    /// The scalars of one side of a face (the cell, a domain ghost, or the wall ghost `2 phi_w - phi_c`).
    #[inline]
    fn side_phi<M: TurbModel<N>>(&self, model: &M, fc: &FaceCells, s: usize, dir: usize) -> [f64; N] {
        let c = fc.cell[s];
        match fc.kind[s] {
            // a jet ghost holds the jet's `[k, omega]` (the jet cell's entry of the scalars: 2D `Kind::Fixed`)
            SideKind::Cell | SideKind::Jet => self.turb[c],
            SideKind::Domain => match self.bc[2 * dir + s] {
                ScalarBc::Inf => model.inf(),
                ScalarBc::Cell => self.turb[c],
            },
            SideKind::Wall => {
                let mut v = self.turb[c];
                for e in 0..N {
                    if let Some(fv) = model.wall_value(e, &self.wall[c]) {
                        v[e] = 2.0 * fv - self.turb[c][e];
                    }
                }
                v
            }
        }
    }

    #[inline]
    fn cell_in(&self, c: usize, phi: [f64; N], grad: SGrad<N>) -> CellIn<N> {
        let g = self.grid;
        let w = &self.w[pidx3(g.ni, g.nj, g.nk, (c % g.ni) as isize, ((c / g.ni) % g.nj) as isize, (c / (g.ni * g.nj)) as isize)];
        let nu = self.mu[c] / w[0];
        let s2 = match self.law_speed(c, w) {
            Some(speed) => {
                let s = strain_law(nu, speed, self.dwall[c]);
                s * s
            }
            None => strain2(&self.grad[c]),
        };
        CellIn { rho: w[0], nu, d: self.dwall[c], phi, grad, s2, mu_t: self.mu_t[c] }
    }

    /// The speed the law of the wall takes in the cell `c` if it takes the law for its strain and its pinned scalars: `law_strain`, a smooth wall cell with a no-slip
    /// face of the law class ([`wall_speed`]); `None` otherwise (a plain wall cell, a rough one, no wall).
    #[inline]
    fn law_speed(&self, c: usize, w: &Prim3) -> Option<f64> {
        if !(self.law_strain && self.wall.get(c).is_some_and(|w| w[1] > 0.0) && self.ks.get(c).is_none_or(|&ks| ks == 0.0)) {
            return None;
        }
        let g = self.grid;
        wall_speed(g, self.dirs, c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj), w, self.law_i_walls)
    }

    /// The pin `(y+, u_tau, nu)` of the law of the wall of a smooth wall cell.
    fn wall_law(&self, c: usize) -> Option<(f64, f64, f64)> {
        let g = self.grid;
        let w = &self.w[pidx3(g.ni, g.nj, g.nk, (c % g.ni) as isize, ((c / g.ni) % g.nj) as isize, (c / (g.ni * g.nj)) as isize)];
        let speed = self.law_speed(c, w)?;
        let nu = self.mu[c] / w[0];
        let u_tau = u_tau_law(nu, speed, self.dwall[c]);
        Some((self.dwall[c] * u_tau / nu, u_tau, nu))
    }
}

/// Kernel 1: Green-Gauss gradient of the scalars of every fluid cell, `phi_f = (phi_c + phi_neighbour) / 2` (ghosts as the face pass; zero in solid cells).
pub fn scalar_gradients<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M) -> Vec<SGrad<N>> {
    let g = cx.grid;
    let (ni, nj) = (g.ni, g.nj);
    let mut out = vec![[[0.0; 3]; N]; ni * nj * g.nk];
    out.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let c = g.cell(i, j, k);
                if cx.solid[c] {
                    continue;
                }
                let pc = cx.turb[c];
                let mut gr = [[0.0; 3]; N];
                for dir in 0..3 {
                    for hi in [false, true] {
                        let mut f = [i, j, k];
                        f[dir] += hi as usize;
                        let Some(fc) = face_cells(g, cx.solid, dir, f[0], f[1], f[2]) else { continue };
                        let nb = cx.side_phi(model, &fc, hi as usize, dir);
                        let (_, face) = face_of(g, dir, f[0], f[1], f[2]);
                        let sgn = if hi { 1.0 } else { -1.0 };
                        for e in 0..N {
                            let pf = 0.5 * (pc[e] + nb[e]);
                            for a in 0..3 {
                                gr[e][a] += sgn * pf * face.n[a] * face.area;
                            }
                        }
                    }
                }
                let inv_v = 1.0 / g.vol[c];
                for e in 0..N {
                    for a in 0..3 {
                        gr[e][a] *= inv_v;
                    }
                }
                pl[j * ni + i] = gr;
            }
        }
    });
    out
}

/// Kernel 2: `F1` and the cross diffusion `X` of every fluid cell.
pub fn blends<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M, tg: &[SGrad<N>]) -> Vec<Blend> {
    let n = cx.turb.len();
    let mut out = vec![Blend::default(); n];
    out.par_iter_mut().enumerate().for_each(|(c, b)| {
        if !cx.solid[c] {
            *b = model.blend(&cx.cell_in(c, cx.turb[c], tg[c]));
        }
    });
    out
}

/// Scalar convection and diffusion data of one face (`l` / `r`: the two sides, lower / upper index).
#[derive(Clone, Copy, Debug)]
pub struct TFace<const N: usize> {
    /// Mass flow `F[0] A` in the positive direction.
    pub m: f64,
    /// Diffusive conductance `G = (mu_f + sigma_f mu_t,f) A / d` per scalar.
    pub g: [f64; N],
    /// Diffusive flux `(mu_f + sigma_f mu_t,f) (g_f . n) A` per scalar in the positive direction.
    pub flux: [f64; N],
    pub l: [f64; N],
    pub r: [f64; N],
}

impl<const N: usize> TFace<N> {
    pub const ZERO: TFace<N> = TFace { m: 0.0, g: [0.0; N], flux: [0.0; N], l: [0.0; N], r: [0.0; N] };
}

/// Kernel 3 (one face): see the module documentation.
fn tface<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M, tg: &[SGrad<N>], blend: &[Blend], dir: usize, i: usize, j: usize, k: usize) -> TFace<N> {
    let (q, face) = face_of(cx.grid, dir, i, j, k);
    let fd = &cx.dirs.family(dir)[q];
    let Some(fc) = face_cells(cx.grid, cx.solid, dir, i, j, k) else { return TFace::ZERO };
    if fd.d == 0.0 {
        return TFace::ZERO;
    }
    let (l, r) = (cx.side_phi(model, &fc, 0, dir), cx.side_phi(model, &fc, 1, dir));
    let m = cx.flux(dir)[q][0] * face.area;
    let ghost = [fc.kind[0] != SideKind::Cell, fc.kind[1] != SideKind::Cell];
    let (mu_f, mut_f) = face_mu_cells(cx.mu, cx.mu_t, cx.wall, fc.cell, ghost, fc.noslip());
    let reflect = |g: [f64; 3]| {
        let d = g[0] * face.n[0] + g[1] * face.n[1] + g[2] * face.n[2];
        [g[0] - 2.0 * d * face.n[0], g[1] - 2.0 * d * face.n[1], g[2] - 2.0 * d * face.n[2]]
    };
    let mut out = TFace { m, g: [0.0; N], flux: [0.0; N], l, r };
    for e in 0..N {
        let [s1, s2] = model.sigmas(e);
        let sigma = 0.5 * (blend[fc.cell[0]].f1 * s1 + (1.0 - blend[fc.cell[0]].f1) * s2 + blend[fc.cell[1]].f1 * s1 + (1.0 - blend[fc.cell[1]].f1) * s2);
        let coef = mu_f + sigma * mut_f;
        let side_g = |s: usize| if fd.sym && ghost[s] { reflect(tg[fc.cell[s]][e]) } else { tg[fc.cell[s]][e] };
        let (gl, gr) = (side_g(0), side_g(1));
        let gb = [0.5 * (gl[0] + gr[0]), 0.5 * (gl[1] + gr[1]), 0.5 * (gl[2] + gr[2])];
        let corr = (r[e] - l[e]) / fd.d - (gb[0] * fd.e[0] + gb[1] * fd.e[1] + gb[2] * fd.e[2]);
        let gn = (gb[0] + corr * fd.e[0]) * face.n[0] + (gb[1] + corr * fd.e[1]) * face.n[1] + (gb[2] + corr * fd.e[2]) * face.n[2];
        out.flux[e] = coef * gn * face.area;
        out.g[e] = coef * face.area / fd.d;
    }
    out
}

/// Kernel 3: scalar face data of every i-, j- and k-face.
pub fn face_data<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M, tg: &[SGrad<N>], blend: &[Blend]) -> [Vec<TFace<N>>; 3] {
    [0, 1, 2].map(|dir| {
        let [n0, n1, n2] = face_shape(cx.grid, dir);
        let mut v = vec![TFace::ZERO; n0 * n1 * n2];
        map_faces(cx.grid, dir, &mut v, |i, j, k, _| tface(cx, model, tg, blend, dir, i, j, k));
        v
    })
}

/// Residual and line coefficients of one cell, per scalar; `[direction][scalar]`.
#[derive(Clone, Copy, Debug)]
pub struct CellTerms<const N: usize> {
    /// `R = rho dphi/dt`.
    pub r: [f64; N],
    pub d0: [f64; N],
    pub lo: [[f64; N]; 3],
    pub dd: [[f64; N]; 3],
    pub up: [[f64; N]; 3],
}

impl<const N: usize> CellTerms<N> {
    const ZERO: CellTerms<N> = CellTerms { r: [0.0; N], d0: [1.0; N], lo: [[0.0; N]; 3], dd: [[0.0; N]; 3], up: [[0.0; N]; 3] };
}

/// Kernel 4: residual `R`, `diag0` and the line coefficients of every fluid cell.
pub fn cell_terms<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M, tg: &[SGrad<N>], blend: &[Blend], tf: &[Vec<TFace<N>>; 3]) -> Vec<CellTerms<N>> {
    let g = cx.grid;
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let mut out = vec![CellTerms::ZERO; ni * nj * nk];
    out.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let c = g.cell(i, j, k);
                if cx.solid[c] {
                    continue;
                }
                let vol = g.vol[c];
                let rho = cx.rho(c);
                let phi = cx.turb[c];
                let (sources, dest) = model.terms(&cx.cell_in(c, phi, tg[c]), &blend[c]);
                let mut o = CellTerms { r: [0.0; N], d0: [0.0; N], lo: [[0.0; N]; 3], dd: [[0.0; N]; 3], up: [[0.0; N]; 3] };
                let mut rate = [0.0; N];
                for dir in 0..3 {
                    let mut f = [i, j, k];
                    let (fl, fh) = {
                        let lo = face_of(g, dir, f[0], f[1], f[2]).0;
                        f[dir] += 1;
                        (lo, face_of(g, dir, f[0], f[1], f[2]).0)
                    };
                    let (lo_f, up_f) = (&tf[dir][fl], &tf[dir][fh]);
                    // a neighbour that is a ghost, a solid cell or across a face without area is not coupled; a ghost that follows its cell (`ScalarBc::Cell`: a
                    // symmetry plane, the axis, zero gradient) is an exact identity ghost = cell, so its face adds nothing to the diagonal either (the frozen
                    // ghosts, `Inf` and walls, keep the diagonal of the 2D module)
                    let mut lf = [i, j, k];
                    let lfc = face_cells(g, cx.solid, dir, lf[0], lf[1], lf[2]);
                    lf[dir] += 1;
                    let ufc = face_cells(g, cx.solid, dir, lf[0], lf[1], lf[2]);
                    let lower = lfc.is_some_and(|fc| fc.kind[0] == SideKind::Cell);
                    let upper = ufc.is_some_and(|fc| fc.kind[1] == SideKind::Cell);
                    let lower_id = lfc.is_some_and(|fc| fc.kind[0] == SideKind::Domain && cx.bc[2 * dir] == ScalarBc::Cell);
                    let upper_id = ufc.is_some_and(|fc| fc.kind[1] == SideKind::Domain && cx.bc[2 * dir + 1] == ScalarBc::Cell);
                    let (a_lo, a_up) = (lo_f.m.max(0.0), (-up_f.m).max(0.0));
                    for e in 0..N {
                        rate[e] += a_lo * (lo_f.l[e] - phi[e]) + a_up * (up_f.r[e] - phi[e]) + up_f.flux[e] - lo_f.flux[e];
                        o.lo[dir][e] = if lower { -(a_lo + lo_f.g[e]) } else { 0.0 };
                        o.up[dir][e] = if upper { -(a_up + up_f.g[e]) } else { 0.0 };
                        o.dd[dir][e] = if lower_id { 0.0 } else { a_lo + lo_f.g[e] } + if upper_id { 0.0 } else { a_up + up_f.g[e] };
                    }
                }
                for e in 0..N {
                    o.r[e] = rate[e] / vol + sources[e];
                    o.d0[e] = rho * vol / cx.dt[c] + rho * vol * dest[e];
                }
                pl[j * ni + i] = o;
            }
        }
    });
    out
}

/// The scalar tridiagonal coefficients of one directional factor, per cell.
struct Factor<const N: usize> {
    lo: Vec<[f64; N]>,
    d: Vec<[f64; N]>,
    up: Vec<[f64; N]>,
}

/// Thomas solve of the line of `n` cells at `base + m * stride`: `lo_m x_{m-1} + d_m x_m + up_m x_{m+1} = rhs_m` for the `N` scalars at once (in place in
/// `rhs`, local to the line).
fn thomas<const N: usize>(n: usize, base: usize, stride: usize, f: &Factor<N>, rhs: &mut [[f64; N]]) {
    let mut cp = vec![[0.0; N]; n];
    for m in 0..n {
        let c = base + m * stride;
        for e in 0..N {
            let (mut den, mut num) = (f.d[c][e], rhs[m][e]);
            if m > 0 {
                den -= f.lo[c][e] * cp[m - 1][e];
                num -= f.lo[c][e] * rhs[m - 1][e];
            }
            cp[m][e] = f.up[c][e] / den;
            rhs[m][e] = num / den;
        }
    }
    for m in (0..n.saturating_sub(1)).rev() {
        for e in 0..N {
            rhs[m][e] -= cp[m][e] * rhs[m + 1][e];
        }
    }
}

/// `x <- A_dir^{-1} x` for every line of direction `dir`.
fn solve_lines<const N: usize>(grid: &Grid3, fac: &Factor<N>, dir: usize, x: &mut [[f64; N]]) {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let (nl, stride, nlines) = match dir {
        0 => (ni, 1, nj * nk),
        1 => (nj, ni, ni * nk),
        _ => (nk, ni * nj, ni * nj),
    };
    let base_of = |l: usize| match dir {
        0 => l * ni,
        1 => (l / ni) * ni * nj + l % ni,
        _ => l,
    };
    let x_in: &[[f64; N]] = x;
    let lines: Vec<Vec<[f64; N]>> = (0..nlines)
        .into_par_iter()
        .map(|l| {
            let base = base_of(l);
            let mut rhs: Vec<[f64; N]> = (0..nl).map(|m| x_in[base + m * stride]).collect();
            thomas(nl, base, stride, fac, &mut rhs);
            rhs
        })
        .collect();
    for (l, line) in lines.iter().enumerate() {
        let base = base_of(l);
        for (m, v) in line.iter().enumerate() {
            x[base + m * stride] = *v;
        }
    }
}

/// Result of an update: the new scalars, the new eddy viscosity, and the density-weighted `L2` norm of the residual `R_e` of every equation over the fluid cells.
pub struct Update<const N: usize> {
    pub turb: Vec<[f64; N]>,
    pub mu_t: Vec<f64>,
    pub rnorm: [f64; N],
}

/// The solves and the clipped update of the scalars from the cell terms of [`cell_terms`] (three factors i, j, k), and the new eddy viscosity.
pub fn solve_and_update<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M, tg: &[SGrad<N>], cells: &[CellTerms<N>]) -> Update<N> {
    let g = cx.grid;
    let n = cells.len();
    let mut x = vec![[0.0; N]; n];
    let mut facs: [Factor<N>; 3] = [0, 1, 2].map(|_| Factor { lo: vec![[0.0; N]; n], d: vec![[1.0; N]; n], up: vec![[0.0; N]; n] });
    for c in 0..n {
        if cx.solid[c] {
            continue;
        }
        let t = &cells[c];
        for e in 0..N {
            x[c][e] = g.vol[c] * t.r[e];
        }
        for (dir, fac) in facs.iter_mut().enumerate() {
            for e in 0..N {
                if dir == 0 {
                    fac.lo[c][e] = t.lo[0][e];
                    fac.up[c][e] = t.up[0][e];
                    fac.d[c][e] = t.d0[e] + t.dd[0][e];
                } else {
                    fac.lo[c][e] = t.lo[dir][e] / t.d0[e];
                    fac.up[c][e] = t.up[dir][e] / t.d0[e];
                    fac.d[c][e] = 1.0 + t.dd[dir][e] / t.d0[e];
                }
            }
        }
    }
    for (dir, fac) in facs.iter().enumerate() {
        solve_lines(g, fac, dir, &mut x);
    }
    let mut turb = vec![[0.0; N]; n];
    let mut mu_t = vec![0.0; n];
    turb.par_iter_mut().zip(mu_t.par_iter_mut()).enumerate().for_each(|(c, (t, m))| {
        if cx.solid[c] {
            return;
        }
        let old = cx.turb[c];
        for e in 0..N {
            t[e] = model.clip(e, old[e], old[e] + x[c][e]);
        }
        if let Some((yp, u_tau, nu)) = cx.wall_law(c) {
            for e in 0..N {
                t[e] = model.wall_cell(e, t[e], yp, u_tau, nu);
            }
        }
        *m = model.eddy_viscosity(&cx.cell_in(c, *t, tg[c]));
    });
    let n_active = cx.solid.iter().filter(|s| !**s).count().max(1) as f64;
    let mut rnorm = [0.0; N];
    for e in 0..N {
        let sum: f64 = cells.par_iter().map(|t| t.r[e] * t.r[e]).sum();
        rnorm[e] = (sum / n_active).sqrt();
    }
    Update { turb, mu_t, rnorm }
}

/// One turbulence update from explicit inputs (see the module documentation): the scalar gradients, the blending functions, the face data, the residual and
/// the line coefficients, the three-factor line solves, the clipped update and the new eddy viscosity.
pub fn update<M: TurbModel<N>, const N: usize>(cx: &TurbIn<N>, model: &M) -> Update<N> {
    let tg = scalar_gradients(cx, model);
    let blend = blends(cx, model, &tg);
    let tf = face_data(cx, model, &tg, &blend);
    let cells = cell_terms(cx, model, &tg, &blend, &tf);
    solve_and_update(cx, model, &tg, &cells)
}

/// The free-stream state of the scalars of every fluid cell and the eddy viscosity of the model there (`rho k / omega` for SST, what `turbulence::init` gives):
/// `(turb, mu_t)`, with the density per cell from `rho`.
pub fn init<M: TurbModel<N>, const N: usize>(model: &M, solid: &[bool], rho: impl Fn(usize) -> f64 + Sync) -> (Vec<[f64; N]>, Vec<f64>) {
    let inf = model.inf();
    let turb = vec![inf; solid.len()];
    let cell = |c: usize| CellIn { rho: rho(c), nu: 0.0, d: f64::INFINITY, phi: inf, grad: [[0.0; 3]; N], s2: 0.0, mu_t: 0.0 };
    let mu_t: Vec<f64> = (0..solid.len()).map(|c| if solid[c] { 0.0 } else { model.eddy_viscosity(&cell(c)) }).collect();
    (turb, mu_t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux3::tests::{random_rotation, rot, Rng};
    use crate::grid3::SolidBox;
    use crate::sst::{Turbulence, Viscous};
    use crate::viscous3::tests::{box_grid, padded, set_solids, solid_mask};
    use crate::viscous3::{cell_pass, face_dirs, face_pass, face_sides3, wall_tractions, SymSides};
    use crate::walldist3::{wall_distance3, wall_faces};

    const SST_RE: f64 = 2.0e3;

    fn visc(re_c: f64) -> Viscous {
        Viscous { re_c, sutherland: 0.368, turbulence: Turbulence::Sst, wall_i_start: 0, roughness: 0.0 }
    }

    /// Mean flow and turbulence inputs of a test case, with everything the update needs computed by the 3D kernels (`viscous3::cell_pass`, `walldist3`,
    /// `wall_state_pass`); the mass flux of a face is the central `(rho u.n)` of its two sides (zero through a wall, whose ghost velocity is reversed).
    struct Case {
        grid: Grid3,
        solid: Vec<bool>,
        w: Vec<Prim3>,
        dirs: FaceDirs3,
        bc: ScalarSides,
        mu: Vec<f64>,
        mu_t: Vec<f64>,
        grad: Vec<Grad3>,
        wall: Vec<[f64; 3]>,
        dwall: Vec<f64>,
        f: [Vec<Cons3>; 3],
        dt: Vec<f64>,
        turb: Vec<[f64; 2]>,
    }

    impl Case {
        #[allow(clippy::too_many_arguments)]
        fn new(grid: Grid3, cells: &[Prim3], sym: SymSides, bc: ScalarSides, ks: &[f64], dt: f64, turb: Vec<[f64; 2]>, re_c: f64) -> Case {
            let solid = solid_mask(&grid);
            let w = padded(&grid, &solid, cells);
            Case::from_w(grid, w, sym, bc, ks, dt, turb, None, re_c)
        }

        /// The case of a padded primitive array `w` with the ghosts the caller chose; `mu_t` the eddy viscosity (`None`: that of the model from the scalars).
        #[allow(clippy::too_many_arguments)]
        fn from_w(
            grid: Grid3,
            w: Vec<Prim3>,
            sym: SymSides,
            bc: ScalarSides,
            ks: &[f64],
            dt: f64,
            turb: Vec<[f64; 2]>,
            mu_t: Option<Vec<f64>>,
            re_c: f64,
        ) -> Case {
            let solid = solid_mask(&grid);
            let dirs = face_dirs(&grid, &solid, &sym);
            let nc = solid.len();
            let (mut grad, mut mu) = (vec![[0.0; 12]; nc], vec![0.0; nc]);
            cell_pass(&visc(re_c), &grid, &solid, &w, &mut grad, &mut mu);
            let dwall = wall_distance3(&grid, &solid, &wall_faces(&grid, &solid));
            let mut wall = vec![[0.0; 3]; nc];
            wall_state_pass(&grid, &solid, &dirs, &w, &mu, &dwall, ks, false, &mut wall);
            let f = [0, 1, 2].map(|dir| {
                let [n0, n1, n2] = face_shape(&grid, dir);
                let mut v = vec![[0.0; 5]; n0 * n1 * n2];
                map_faces(&grid, dir, &mut v, |i, j, k, q| match face_sides3(&grid, &solid, &w, dir, i, j, k) {
                    Some(s) => {
                        let n = face_of(&grid, dir, i, j, k).1.n;
                        let mf = |w: &Prim3| w[0] * (w[1] * n[0] + w[2] * n[1] + w[3] * n[2]);
                        // a far-field ghost carries the flux of its cell (a symmetry ghost is the reflected state, a wall ghost the reversed velocity)
                        let st = |sd: &crate::viscous3::Side3| {
                            if sd.ghost && !s.noslip && !dirs.family(dir)[q].sym {
                                w[pidx3(
                                    grid.ni,
                                    grid.nj,
                                    grid.nk,
                                    (sd.cell % grid.ni) as isize,
                                    ((sd.cell / grid.ni) % grid.nj) as isize,
                                    (sd.cell / (grid.ni * grid.nj)) as isize,
                                )]
                            } else {
                                sd.w
                            }
                        };
                        [0.5 * (mf(&st(&s.l)) + mf(&st(&s.r))), 0.0, 0.0, 0.0, 0.0]
                    }
                    None => [0.0; 5],
                });
                v
            });
            let model = Sst { re_c };
            let mu_t = mu_t.unwrap_or_else(|| {
                (0..nc)
                    .map(|c| {
                        if solid[c] {
                            return 0.0;
                        }
                        let rho = w
                            [pidx3(grid.ni, grid.nj, grid.nk, (c % grid.ni) as isize, ((c / grid.ni) % grid.nj) as isize, (c / (grid.ni * grid.nj)) as isize)]
                            [0];
                        model.eddy_viscosity(&CellIn { rho, nu: mu[c] / rho, d: dwall[c], phi: turb[c], grad: [[0.0; 3]; 2], s2: strain2(&grad[c]), mu_t: 0.0 })
                    })
                    .collect()
            });
            let dt = vec![dt; nc];
            Case { grid, solid, w, dirs, bc, mu, mu_t, grad, wall, dwall, f, dt, turb }
        }

        fn input(&self) -> TurbIn<'_, 2> {
            TurbIn {
                grid: &self.grid,
                solid: &self.solid,
                w: &self.w,
                dirs: &self.dirs,
                bc: &self.bc,
                mu: &self.mu,
                mu_t: &self.mu_t,
                grad: &self.grad,
                wall: &self.wall,
                ks: &[],
                law_strain: false,
                law_i_walls: false,
                dwall: &self.dwall,
                fi: &self.f[0],
                fj: &self.f[1],
                fk: &self.f[2],
                dt: &self.dt,
                turb: &self.turb,
            }
        }
    }

    // ---- the wall treatment as a pure function -----------------------------------------------------------------------------------------------------

    /// The Newton inversion of the law of the wall reproduces the friction velocity it was built from, at every `y+` from the viscous sublayer to `1e4`.
    #[test]
    fn law_of_the_wall_inversion_round_trips() {
        let nu = 1.3e-6;
        for yp in [0.01, 0.1, 0.5, 1.0, 3.0, 5.0, 10.0, 15.0, 30.0, 60.0, 100.0, 300.0, 1e3, 1e4] {
            let u_star = 0.37;
            let d = yp * nu / u_star;
            let u = u_star * law_uplus(yp);
            let got = u_tau_law(nu, u, d);
            assert!((got / u_star - 1.0).abs() < 1e-11, "y+ {yp}: {got} vs {u_star}");
        }
        assert_eq!(u_tau_law(nu, 0.0, 1e-4), 0.0);
    }

    /// `reichardt_duplus` / `law_duplus` are the derivatives of `reichardt_uplus` / `law_uplus`, and `strain_law` is the wall shear rate `U / d` in the sublayer and `u_tau / (kappa d)` in the log layer.
    #[test]
    fn law_of_the_wall_strain() {
        for yp in [0.1, 1.0, 5.0, 15.0, 30.0, 300.0] {
            let h = 1e-5 * yp;
            let fd = (reichardt_uplus(yp + h) - reichardt_uplus(yp - h)) / (2.0 * h);
            assert!((reichardt_duplus(yp) - fd).abs() < 1e-8 * (1.0 + fd), "y+ {yp}: {} vs {fd}", reichardt_duplus(yp));
            let fd = (law_uplus(yp + h) - law_uplus(yp - h)) / (2.0 * h);
            assert!((law_duplus(yp) - fd).abs() < 1e-8 * (1.0 + fd), "law y+ {yp}: {} vs {fd}", law_duplus(yp));
        }
        let nu = 1.0e-6;
        for (yp, tol) in [(0.1, 2e-3), (300.0, 2.5e-2)] {
            let u_star = 0.3;
            let d = yp * nu / u_star;
            let speed = u_star * law_uplus(yp);
            let s = strain_law(nu, speed, d);
            let want = if yp < 1.0 { speed / d } else { u_star / (KAPPA * d) };
            assert!((s / want - 1.0).abs() < tol, "y+ {yp}: {s} vs {want}");
        }
    }

    /// The `k` cap of a wall-adjacent cell: untouched where the layer is resolved (`y+ <= 2`: weight `<= 4e-3`), never a raise, the log-layer value `k_wall` where the cell is in the
    /// buffer / log layer (`y+ >= 8`), `omega` never touched.
    #[test]
    fn k_cap_of_the_wall_cell() {
        let sst = Sst { re_c: 2.5e7 };
        let (u_tau, nu) = (0.01, 4.0e-8);
        let k_big = 31.0 * u_tau * u_tau;
        assert!((sst.wall_cell(0, k_big, 0.5, u_tau, nu) / k_big - 1.0).abs() < 1e-6);
        assert!((sst.wall_cell(0, k_big, 2.0, u_tau, nu) / k_big - 1.0).abs() < 5e-3 * (1.0 - k_wall(u_tau, 2.0) / k_big) + 1e-12);
        for yp in [8.0, 15.0, 40.0] {
            let got = sst.wall_cell(0, k_big, yp, u_tau, nu);
            // weight 0.996 at y+ = 8 (0.4 % of the solved value stays: 0.12 u_tau^2 on 0.55), 1 - 3e-5 at 15
            let tol = if yp < 10.0 { 0.25 } else { 0.01 };
            assert!(got >= k_wall(u_tau, yp) && got / k_wall(u_tau, yp) - 1.0 < tol, "y+ {yp}: {got} vs {}", k_wall(u_tau, yp));
            let small = 1e-3 * k_wall(u_tau, yp);
            assert_eq!(sst.wall_cell(0, small, yp, u_tau, nu), small, "a cap, never a raise");
        }
        assert_eq!(sst.wall_cell(1, 123.0, 15.0, u_tau, nu), 123.0);
    }

    /// `y+` behaviour of the smooth wall state against the laws of the wall at `y+ = 0.5, 1, 5, 15, 30, 100` (printed by `cargo test -- --nocapture`): the wall shear
    /// `rho u_tau^2` is the viscous `rho nu U / d` in the sublayer (`mu_t,w = 0` up to `y+ ~ 1`, the 2D wall state), the log-law value at `y+ >= 30`; `omega_w` is Menter's wall
    /// value `60 nu / (beta1 d^2)` up to `y+ ~ 1`, the log-layer `u_tau / (sqrt(beta*) kappa d)` at `y+ >= 30` and never below `omega_log`; `k_w` is 0 in the sublayer and
    /// `u_tau^2 / sqrt(beta*)` in the log layer. The Esch-Menter 4th-power blend `u_tau_blend` is printed next to it.
    #[test]
    fn wall_state_follows_the_sublayer_and_the_log_layer() {
        let (rho, nu) = (1.2, 1.0e-6);
        println!("  y+   tau_w/tau_visc  tau_w/tau_law  tau_blend/tau_law  omega_w/omega_2D  omega_w/omega_log  k_w+  mu_t,w/mu");
        for yp in [0.5, 1.0, 5.0, 15.0, 30.0, 100.0] {
            let (u_star, d) = (0.2, yp * 1.0e-6 / 0.2);
            let u = u_star * law_uplus(yp);
            let s = wall_state_smooth(rho, nu, u, d);
            let tau = s.u_tau * s.u_tau;
            let tau_visc = nu * u / d;
            let tau_log = {
                // the log law U+ = ln(E y+) / kappa at this speed: u_log by fixed point
                let mut ul = u_star;
                for _ in 0..50 {
                    ul = KAPPA * u / (LOG_E * d * ul / nu).ln();
                }
                ul * ul
            };
            let tau_blend = u_tau_blend(nu, u, d).powi(2);
            let om_2d = 60.0 * nu / (BETA1 * d * d);
            let om_log = s.u_tau / (BETA_STAR.sqrt() * KAPPA * d);
            println!(
                "{yp:5}   {:.4}   {:.4}   {:.4}    {:.4}   {:.4}   {:.4}   {:.3}",
                tau / tau_visc,
                tau / (u_star * u_star),
                tau_blend / (u_star * u_star),
                s.omega_w / om_2d,
                s.omega_w / om_log,
                s.k_w / tau,
                s.mu_t_w / (rho * nu)
            );
            assert!((tau / (u_star * u_star) - 1.0).abs() < 1e-10);
            assert!(s.omega_w >= om_log * (1.0 - 1e-12));
            if yp <= 1.0 {
                // the 2D wall state: mu_t,w = 0, k_w ~ 0, omega_w within 0.5 % of 60 nu / (beta1 d^2); the wall shear within 1 % of the viscous one
                assert_eq!(s.mu_t_w, 0.0, "y+ {yp}");
                assert!(s.k_w < 1e-3 * tau, "y+ {yp}: k_w+ {}", s.k_w / tau);
                assert!((s.omega_w / om_2d - 1.0).abs() < 5e-3, "y+ {yp}: omega_w / omega_2D = {}", s.omega_w / om_2d);
                assert!((tau / tau_visc - 1.0).abs() < 1e-2, "y+ {yp}");
            }
            if yp <= 5.0 {
                assert!((tau / tau_visc - 1.0).abs() < 0.1, "y+ {yp}: {}", tau / tau_visc);
            }
            if yp >= 30.0 {
                assert!((tau / tau_log - 1.0).abs() < 0.04, "y+ {yp}: the law of the wall against the log law {}", tau / tau_log);
                assert!((s.omega_w / om_log - 1.0).abs() < 0.06, "y+ {yp}: omega_w / omega_log = {}", s.omega_w / om_log);
                assert!((s.k_w / tau * BETA_STAR.sqrt() - 1.0).abs() < 0.07, "y+ {yp}: k_w+ sqrt(beta*) = {}", s.k_w / tau * BETA_STAR.sqrt());
            }
        }
    }

    /// `ks > 0` takes `turbulence::aupoix_wall` verbatim (state and `mu_t,w = rho nu_t,w`, `u_tau^2 = (nu + nu_t,w) U / d`); a tiny `ks` is its smooth limit,
    /// the 2D wall state `[0, 60 nu / (beta1 d^2), 0]`; `ks = 0` is the automatic treatment; at rest everything is finite.
    #[test]
    fn rough_wall_state_is_aupoix_verbatim() {
        let (rho, nu, d, speed) = (0.9, 1.0e-6, 2.0e-4, 1.3);
        for ks in [1e-5, 1e-4, 1e-3] {
            let s = wall_state(rho, nu, speed, d, ks);
            let [k_w, om_w, nut] = aupoix_wall(nu, speed, d, ks);
            assert_eq!((s.k_w, s.omega_w, s.mu_t_w), (k_w, om_w, rho * nut));
            assert!((s.u_tau * s.u_tau - (nu + nut) * speed / d).abs() < 1e-15);
        }
        let tiny = wall_state(rho, nu, speed, d, 1e-12);
        assert_eq!((tiny.k_w, tiny.mu_t_w), (0.0, 0.0));
        assert_eq!(tiny.omega_w, 60.0 * nu / (BETA1 * d * d));
        assert_eq!(wall_state(rho, nu, speed, d, 0.0), wall_state_smooth(rho, nu, speed, d));
        for ks in [0.0, 1e-4] {
            let s = wall_state(rho, nu, 0.0, d, ks);
            assert!(s.u_tau.is_finite() && s.k_w.is_finite() && s.omega_w.is_finite() && s.mu_t_w.is_finite() && s.omega_w > 0.0);
        }
    }

    /// The wall state of the cells of a box with solid rows: one state per wall cell (zero elsewhere), the roughness taken per cell, and a junction cell (two wall
    /// faces) takes the distance of the nearest wall for both.
    #[test]
    fn wall_state_pass_takes_ks_per_cell_and_the_nearest_wall_at_a_junction() {
        let (n, h) = ([3, 5, 5], [0.01, 0.0004, 0.001]);
        let mut g = box_grid(n, h);
        // walls: y = 0 (solid row j = 0) and z = 0 (solid layer k = 0); the cell (·, 1, 1) touches both
        set_solids(&mut g, vec![SolidBox { i: 0..3, j: 0..1, k: 0..5 }, SolidBox { i: 0..3, j: 1..5, k: 0..1 }]);
        let solid = solid_mask(&g);
        let dirs = face_dirs(&g, &solid, &[false; 6]);
        let nc = solid.len();
        let cells = vec![[1.0, 0.8, 0.0, 0.0, 0.7]; nc];
        let w = padded(&g, &solid, &cells);
        let mu = vec![1.0e-6; nc];
        let dwall = wall_distance3(&g, &solid, &wall_faces(&g, &solid));
        let mut ks = vec![0.0; nc];
        ks[g.cell(1, 1, 1)] = 1e-4; // rough junction cell
        ks[g.cell(1, 2, 1)] = 0.0; // smooth: a cell above the z wall only (j = 2 is beside no y wall)
        let mut out = vec![[0.0; 3]; nc];
        wall_state_pass(&g, &solid, &dirs, &w, &mu, &dwall, &ks, false, &mut out);
        let st = |c: usize, k: f64| {
            let s = wall_state(1.0, 1.0e-6, 0.8, dwall[c], k);
            [s.k_w, s.omega_w, s.mu_t_w]
        };
        // junction (1, 1, 1): distance to y = 0 is 0.0002 (h_y / 2), to z = 0 is 0.0005: the nearest wall is y
        let jn = g.cell(1, 1, 1);
        assert!((dwall[jn] - 0.5 * h[1]).abs() < 1e-15);
        assert_eq!(out[jn], st(jn, 1e-4));
        // a cell with a wall face only on z, smooth
        let cz = g.cell(1, 2, 1);
        assert!((dwall[cz] - 0.5 * h[2]).abs() < 1e-15);
        assert_eq!(out[cz], st(cz, 0.0));
        // a cell far from every wall, a solid cell
        assert_eq!(out[g.cell(1, 3, 3)], [0.0; 3]);
        assert_eq!(out[g.cell(1, 0, 2)], [0.0; 3]);
    }

    /// The wall classes: a cell with only an `i` wall (the front slab, a base plane) gets the plain state (speed 0 in `wall_state`: `k_w = 0`, `mu_t,w = 0`, Menter's `omega_w`)
    /// unless `law_i_walls`, when it takes the law at the speed tangential to that wall; a cell with a `j` wall takes the law at the speed tangential to the `j` wall whether
    /// or not it also has an `i` wall (the largest tangential speed of the law-class faces).
    #[test]
    fn i_walls_get_the_plain_wall_state_unless_the_switch_is_on() {
        let (n, h) = ([5, 4, 3], [0.01, 0.0004, 0.001]);
        let mut g = box_grid(n, h);
        // an `i` wall: the slab i < 2; a streamwise wall: the floor under the fluid at i >= 2 (cell (2, 1, 1) has both)
        set_solids(&mut g, vec![SolidBox { i: 0..2, j: 0..4, k: 0..3 }, SolidBox { i: 2..5, j: 0..1, k: 0..3 }]);
        let solid = solid_mask(&g);
        let dirs = face_dirs(&g, &solid, &[false; 6]);
        let nc = solid.len();
        let w = padded(&g, &solid, &vec![[1.0, 0.8, 0.3, 0.0, 0.7]; nc]);
        let mu = vec![1.0e-6; nc];
        let dwall = wall_distance3(&g, &solid, &wall_faces(&g, &solid));
        let state = |c: usize, speed: f64| {
            let s = wall_state(1.0, 1.0e-6, speed, dwall[c], 0.0);
            [s.k_w, s.omega_w, s.mu_t_w]
        };
        let (only_i, floor, both) = (g.cell(2, 2, 1), g.cell(3, 1, 1), g.cell(2, 1, 1));
        let cell = w[pidx3(n[0], n[1], n[2], 2, 2, 1)];
        assert_eq!(wall_speed(&g, &dirs, 2, 2, 1, &cell, false), None);
        assert!((wall_speed(&g, &dirs, 2, 2, 1, &cell, true).unwrap() - 0.3).abs() < 1e-14, "tangential to the i wall: the v component");
        assert!((wall_speed(&g, &dirs, 3, 1, 1, &cell, false).unwrap() - 0.8).abs() < 1e-14, "tangential to the floor: the u component");
        let mut out = vec![[0.0; 3]; nc];
        wall_state_pass(&g, &solid, &dirs, &w, &mu, &dwall, &[], false, &mut out);
        let same = |a: [f64; 3], b: [f64; 3]| a.iter().zip(&b).all(|(x, y)| (x - y).abs() <= 1e-12 * (x.abs() + y.abs()));
        assert!(same(out[only_i], state(only_i, 0.0)), "{:?}", out[only_i]);
        assert_eq!(out[only_i][0], 0.0);
        assert_eq!(out[only_i][2], 0.0);
        let menter = 60.0 * 1.0e-6 / (BETA1 * dwall[only_i] * dwall[only_i]);
        assert!((out[only_i][1] / menter - 1.0).abs() < 1e-12, "Menter's 2D value {} vs {menter}", out[only_i][1]);
        assert!(same(out[floor], state(floor, 0.8)), "{:?}", out[floor]);
        assert!(same(out[both], state(both, 0.8)), "the i wall does not enter the speed of a cell with a streamwise wall: {:?}", out[both]);
        wall_state_pass(&g, &solid, &dirs, &w, &mu, &dwall, &[], true, &mut out);
        assert!(same(out[only_i], state(only_i, 0.3)), "{:?}", out[only_i]);
        assert!(same(out[floor], state(floor, 0.8)));
        assert!(same(out[both], state(both, 0.8)));
    }

    /// The friction drag is the momentum flux through the wall faces: with the wall state of the automatic treatment at `y+ = 15`, the viscous flux of `viscous3` through
    /// every wall face and `wall_tractions` give `tau_w = rho u_tau^2` along the flow (to roundoff), with the force on the wall pointing along the flow, and the sum over the
    /// faces is `rho u_tau^2 A`. Without the wall state (`mu_t,w = 0`) the same flux is the viscous `mu U / d`, `y+ = 15` too small by the factor `U+ / y+`.
    #[test]
    fn wall_shear_is_the_momentum_flux_through_the_wall_faces() {
        let re_c = 1.0e6;
        let (n, h) = ([4, 6, 3], [0.02, 0.0002, 0.02]);
        let mut g = box_grid(n, h);
        set_solids(&mut g, vec![SolidBox { i: 0..4, j: 0..1, k: 0..3 }]);
        let solid = solid_mask(&g);
        let nc = solid.len();
        let v = visc(re_c);
        let (rho, p) = (1.0, 1.0 / crate::gas3::GAMMA);
        let nu = v.mu(1.0) / rho;
        let d = 0.5 * h[1];
        let u_star = 15.0 * nu / d;
        let u1 = u_star * law_uplus(15.0);
        // a profile that grows away from the wall; the first cell (j = 1) sits at y+ = 15
        let cells: Vec<Prim3> = (0..nc).map(|c| [rho, u1 * (1.0 + 0.4 * ((c / n[0]) % n[1]) as f64 - 0.4), 0.0, 0.0, p]).collect();
        let w = padded(&g, &solid, &cells);
        let dirs = face_dirs(&g, &solid, &[false; 6]);
        let (mut grad, mut mu) = (vec![[0.0; 12]; nc], vec![0.0; nc]);
        cell_pass(&v, &g, &solid, &w, &mut grad, &mut mu);
        let dwall = wall_distance3(&g, &solid, &wall_faces(&g, &solid));
        let mut wall = vec![[0.0; 3]; nc];
        wall_state_pass(&g, &solid, &dirs, &w, &mu, &dwall, &[], false, &mut wall);
        let mu_t = vec![0.0; nc];
        let run = |wall: &[[f64; 3]]| {
            let (mut fi, mut fj, mut fk) = (vec![[0.0; 5]; g.iface.len()], vec![[0.0; 5]; g.jface.len()], vec![[0.0; 5]; g.kface.len()]);
            face_pass(&g, &solid, &w, &dirs, &grad, &mu, &mu_t, wall, &mut fi, &mut fj, &mut fk);
            wall_tractions(&g, &solid, &dirs, &fi, &fj, &fk)
        };
        let tr = run(&wall);
        assert_eq!(tr.len(), n[0] * n[2]);
        let u_tau = wall_state(rho, nu, u1, d, 0.0).u_tau;
        assert!((u_tau / u_star - 1.0).abs() < 1e-10, "the first cell is at y+ = 15");
        let mut total = 0.0;
        for t in &tr {
            assert_eq!(t.n, [0.0, -1.0, 0.0], "the wall normal points out of the fluid into the solid, i.e. -y here: {:?}", t.n);
            let shear = wall_shear_vector(rho, u_tau, [u1, 0.0, 0.0], t.n);
            assert!((t.tau[0] - shear[0]).abs() < 1e-12 * shear[0], "{:?} vs {:?}", t.tau, shear);
            assert!(t.tau[1].abs() < 1e-15 && t.tau[2].abs() < 1e-15);
            assert!(t.tau[0] > 0.0, "the fluid drags the wall along the flow");
            total += t.tau[0] * t.area;
        }
        let area = h[0] * h[2] * (n[0] * n[2]) as f64;
        assert!((total / (rho * u_tau * u_tau * area) - 1.0).abs() < 1e-12);
        // without the wall state the same machinery gives the viscous shear mu U / d
        let tr0 = run(&[]);
        let visc_shear = v.mu(1.0) * u1 / d;
        for t in &tr0 {
            assert!((t.tau[0] / visc_shear - 1.0).abs() < 1e-12);
        }
        assert!((tr[0].tau[0] / tr0[0].tau[0] - 15.0 / law_uplus(15.0)).abs() < 1e-9);
    }

    // ---- the update -------------------------------------------------------------------------------------------------------------------------------

    const ALL_INF: ScalarSides = [ScalarBc::Inf; 6];

    /// A jittered grid with a solid block, a smooth state with random perturbation and positive random scalars: the generic test case.
    fn random_case(rng: &mut Rng, ks: f64) -> (Vec<[f64; 3]>, [usize; 3], Vec<Prim3>, Vec<[f64; 2]>, Vec<f64>) {
        let n = [8, 7, 6];
        let mut nodes = vec![];
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    let jit = |r: &mut Rng| r.range(-0.1, 0.1);
                    nodes.push([0.05 * i as f64 + 0.05 * jit(rng), 0.04 * j as f64 + 0.04 * jit(rng) + 0.01 * i as f64, 0.05 * k as f64 + 0.05 * jit(rng)]);
                }
            }
        }
        let nc = n[0] * n[1] * n[2];
        let cells: Vec<Prim3> =
            (0..nc).map(|_| [rng.range(0.8, 1.2), rng.range(-0.4, 0.9), rng.range(-0.3, 0.3), rng.range(-0.3, 0.3), rng.range(0.6, 1.1)]).collect();
        let turb: Vec<[f64; 2]> = (0..nc).map(|_| [10f64.powf(rng.range(-6.0, -2.0)), 10f64.powf(rng.range(0.0, 2.0))]).collect();
        let ks = (0..nc).map(|c| if c % 3 == 0 { ks } else { 0.0 }).collect();
        (nodes, n, cells, turb, ks)
    }

    fn grid_with_solid(nodes: &[[f64; 3]], n: [usize; 3]) -> Grid3 {
        let mut g = Grid3::try_from_nodes(
            n[0],
            n[1],
            n[2],
            nodes.iter().map(|p| p[0]).collect(),
            nodes.iter().map(|p| p[1]).collect(),
            nodes.iter().map(|p| p[2]).collect(),
        )
        .unwrap();
        set_solids(&mut g, vec![SolidBox { i: 3..5, j: 0..3, k: 2..4 }]);
        g
    }

    /// A uniform stream with the free-stream scalars and `Inf` ghosts everywhere has no transport residual (`R` is exactly the local sources) and the update stays
    /// positive and only decays (the sources); the decrement is uniform in space away from the boundaries (a frozen far-field ghost adds the inflow mass flow and the
    /// conductance to the diagonal of the boundary cells, so these damp the update, the inflow ones by `1 / (1 + a dt / rho V)`: the corner cell here by half); `mu_t` is
    /// the `rho k / omega` of the new scalars.
    #[test]
    fn free_stream_has_no_transport_residual_and_stays_uniform() {
        let model = Sst { re_c: SST_RE };
        let g = box_grid([7, 6, 5], [0.1, 0.12, 0.09]);
        let nc = 7 * 6 * 5;
        let cells = vec![[1.1, 0.8, 0.1, -0.05, 0.9]; nc];
        let inf = model.inf();
        let case = Case::new(g, &cells, [false; 6], ALL_INF, &[], 0.1, vec![inf; nc], SST_RE);
        let cx = case.input();
        let tg = scalar_gradients(&cx, &model);
        let blend = blends(&cx, &model, &tg);
        let tf = face_data(&cx, &model, &tg, &blend);
        let terms = cell_terms(&cx, &model, &tg, &blend, &tf);
        let rho = 1.1;
        for t in &terms {
            // sources of the free stream: no production (S = 0), decay -beta* rho omega k and -beta rho omega^2 (F1 = 0: the cells have no wall: beta2 / (1 - F1) X = 0)
            let (src, _) = model.terms(
                &CellIn { rho, nu: 0.0, d: f64::INFINITY, phi: inf, grad: [[0.0; 3]; 2], s2: 0.0, mu_t: rho * inf[0] / inf[1] },
                &Blend { f1: 0.0, x: 0.0 },
            );
            for e in 0..2 {
                assert!((t.r[e] - src[e]).abs() <= 1e-12 * src[e].abs(), "R_{e} = {:e} vs sources {:e}", t.r[e], src[e]);
            }
        }
        let up = solve_and_update(&cx, &model, &tg, &terms);
        let mid = case.grid.cell(3, 3, 2);
        for c in 0..nc {
            let (i, j, k) = (c % 7, (c / 7) % 6, c / 42);
            for e in 0..2 {
                assert!(up.turb[c][e] > 0.0 && up.turb[c][e] <= inf[e], "scalar {e} grew or vanished: {:e} from {:e}", up.turb[c][e], inf[e]);
                let (dec, dec_mid) = (inf[e] - up.turb[c][e], inf[e] - up.turb[mid][e]);
                if (2..5).contains(&i) && (2..4).contains(&j) && (1..4).contains(&k) {
                    assert!((dec - dec_mid).abs() <= 0.1 * dec_mid, "decrement of scalar {e} at cell {c}: {dec:e} vs {dec_mid:e}");
                }
            }
            assert!((up.mu_t[c] / (rho * up.turb[c][0] / up.turb[c][1]) - 1.0).abs() < 1e-12);
        }
    }

    /// The update of random positive scalars on a jittered grid with a solid block and rough and smooth wall cells keeps `k`, `omega` positive and at least `0.1` of
    /// their old values, `mu_t >= 0` and finite, zero in the solid cells, and is deterministic.
    #[test]
    fn update_keeps_the_scalars_positive_and_finite() {
        let mut rng = Rng::new(11);
        for ks in [0.0, 1e-3] {
            let (nodes, n, cells, turb, ksv) = random_case(&mut rng, ks);
            let case = Case::new(
                grid_with_solid(&nodes, n),
                &cells,
                [false; 6],
                [ScalarBc::Inf, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Cell],
                &ksv,
                0.5,
                turb,
                SST_RE,
            );
            let model = Sst { re_c: SST_RE };
            let cx = case.input();
            let a = update(&cx, &model);
            let b = update(&cx, &model);
            assert_eq!(a.turb, b.turb);
            for c in 0..case.solid.len() {
                if case.solid[c] {
                    assert_eq!((a.turb[c], a.mu_t[c]), ([0.0; 2], 0.0));
                    continue;
                }
                for e in 0..2 {
                    assert!(
                        a.turb[c][e].is_finite() && a.turb[c][e] >= 0.1 * case.turb[c][e],
                        "cell {c} scalar {e}: {:e} from {:e}",
                        a.turb[c][e],
                        case.turb[c][e]
                    );
                }
                assert!(a.mu_t[c].is_finite() && a.mu_t[c] >= 0.0 && a.mu_t[c] <= 1e5 / SST_RE);
            }
            assert!(a.rnorm.iter().all(|r| r.is_finite() && *r >= 0.0));
        }
    }

    /// The update does not depend on the orientation of the coordinate axes: rotating (or mirroring) the grid and the velocity vectors leaves `k`, `omega` and `mu_t`
    /// unchanged to roundoff (the scalar gradients, the strain, the wall distance and the wall treatment are all invariants; line solves run along the grid lines).
    #[test]
    fn update_is_invariant_under_rotations_and_mirror_images() {
        let mut rng = Rng::new(5);
        let (nodes, n, cells, turb, ksv) = random_case(&mut rng, 5e-4);
        let model = Sst { re_c: SST_RE };
        let bc = [ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Cell, ScalarBc::Inf, ScalarBc::Inf, ScalarBc::Cell];
        let base = Case::new(grid_with_solid(&nodes, n), &cells, [false; 6], bc, &ksv, 0.3, turb.clone(), SST_RE);
        let a = update(&base.input(), &model);
        for trial in 0..4 {
            let mut q = random_rotation(&mut rng);
            if trial >= 2 {
                // reflect about x = 0 as well
                for r in q.iter_mut() {
                    r[0] = -r[0];
                }
            }
            let nodes_b: Vec<[f64; 3]> = nodes.iter().map(|p| rot(&q, *p)).collect();
            let cells_b: Vec<Prim3> = cells
                .iter()
                .map(|c| {
                    let v = rot(&q, [c[1], c[2], c[3]]);
                    [c[0], v[0], v[1], v[2], c[4]]
                })
                .collect();
            let case = Case::new(grid_with_solid(&nodes_b, n), &cells_b, [false; 6], bc, &ksv, 0.3, turb.clone(), SST_RE);
            let b = update(&case.input(), &model);
            for c in 0..case.solid.len() {
                for e in 0..2 {
                    assert!(
                        (a.turb[c][e] - b.turb[c][e]).abs() <= 1e-9 * a.turb[c][e].abs(),
                        "trial {trial} cell {c} scalar {e}: {:e} vs {:e}",
                        a.turb[c][e],
                        b.turb[c][e]
                    );
                }
                assert!((a.mu_t[c] - b.mu_t[c]).abs() <= 1e-9 * a.mu_t[c].abs() + 1e-30, "trial {trial} cell {c} mu_t");
            }
        }
    }

    // ---- parity with the 2D update on a one-cell wedge ---------------------------------------------------------------------------------------------

    mod parity {
        use super::*;
        use crate::gas::{freestream, prim_to_cons};
        use crate::limiter::Limiter;
        use crate::solver::{Bc, Config, Solver};
        use crate::viscous::{iface_sides, jface_sides};
        use crate::viscous3::wedge::{col_x, smooth_state, wedge};

        const DT: f64 = 0.05;
        const WEDGE_BC: ScalarSides = [ScalarBc::Inf, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Cell];
        const WEDGE_SYM: SymSides = [false, false, false, false, true, true];

        /// Smooth positive `[k, omega]` and eddy viscosity fields of `(x, r)`.
        fn kw(x: f64, r: f64) -> [f64; 2] {
            [2e-4 * (1.0 + 0.5 * (1.7 * x).sin() * (2.0 * r).cos()), 30.0 * (1.0 + 0.4 * (1.3 * x + r).cos())]
        }

        fn mut_of(x: f64, r: f64) -> f64 {
            1.5e-3 * (1.0 + 0.5 * (x + r).sin())
        }

        /// One update of the 2D `turbulence::update` and of the 3D wedge on the same state, `[dk, domega, mu_t]` relative `L2` and `Linf` difference over the inner
        /// cells (>= 2 cells from the domain boundaries, > 8 from the axis, as the viscous parity): the changes `phi_new - phi_old` of the two scalars and the new `mu_t`.
        /// `auto`: the wall state of the 3D case is the automatic treatment (else the 2D wall state `[0, 60 nu / (beta1 d^2), 0]`).
        pub(super) fn run(ni: usize, nj: usize, wall: bool, deg: f64, auto: bool) -> ([f64; 3], [f64; 3]) {
            let f = smooth_state(wall.then(|| col_x(ni / 2, ni)));
            let wd = wedge(ni, nj, wall, deg.to_radians(), true, &f);
            let g2 = wedge(ni, nj, wall, deg.to_radians(), true, &f).g2;
            let blk = wd.blk;
            // ---- 2D
            let cfg = Config {
                cfl: 0.5,
                limiter: Limiter::VanAlbada,
                freestream: freestream(0.5),
                bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
                first_order_iters: 0,
            };
            let mut s = Solver::new(g2, cfg);
            s.set_block(blk);
            // without a body the `j = 0` slip wall of the 2D module would be a no-slip plate from `wall_i_start`: put its start beyond the grid (the axis is no wall)
            s.set_viscous(Some(Viscous { wall_i_start: if wall { 0 } else { ni }, ..visc(SST_RE) }));
            let nc = ni * nj;
            let fluid = |c: usize| !blk.is_some_and(|b| b.solid(c % ni, c / ni));
            for c in 0..nc {
                if fluid(c) {
                    s.u[c] = prim_to_cons(&f(s.grid.xc[c], s.grid.rc[c]));
                }
            }
            s.eval_residual(true);
            let walls = s.walls();
            let mass = |nx: f64, nr: f64, fs: &crate::viscous::FaceSides, ni: usize, w: &[crate::gas::Prim]| {
                // a far-field ghost carries the flux of its cell
                let st = |sd: &crate::viscous::Side| {
                    if sd.ghost && !fs.noslip {
                        w[crate::solver::pidx(ni, (sd.cell % ni) as isize, (sd.cell / ni) as isize)]
                    } else {
                        sd.w
                    }
                };
                let (a, b) = (st(&fs.l), st(&fs.r));
                0.5 * (a[0] * (a[1] * nx + a[2] * nr) + b[0] * (b[1] * nx + b[2] * nr))
            };
            for j in 0..nj {
                for i in 0..=ni {
                    let q = j * (ni + 1) + i;
                    s.fi[q][0] = iface_sides(&s.grid, &s.w, &walls, i, j).map_or(0.0, |sd| mass(s.grid.iface[q].nx, s.grid.iface[q].nr, &sd, ni, &s.w));
                }
            }
            for j in 0..=nj {
                for i in 0..ni {
                    let q = j * ni + i;
                    s.fj[q][0] = jface_sides(&s.grid, &s.w, &walls, i, j).map_or(0.0, |sd| mass(s.grid.jface[q].nx, s.grid.jface[q].nr, &sd, ni, &s.w));
                }
            }
            let turb0: Vec<[f64; 2]> = (0..nc).map(|c| if fluid(c) { kw(s.grid.xc[c], s.grid.rc[c]) } else { [0.0; 2] }).collect();
            for c in 0..nc {
                if fluid(c) {
                    s.turb[c] = turb0[c];
                    s.mu_t[c] = mut_of(s.grid.xc[c], s.grid.rc[c]);
                    s.dt[c] = DT;
                }
            }
            crate::turbulence::update(&mut s);
            // ---- 3D
            let g3 = &wd.g3;
            let own = |c: usize| (g3.xc[c][0], g3.xc[c][1].hypot(g3.xc[c][2]));
            let turb3: Vec<[f64; 2]> = (0..nc).map(|c| if fluid(c) { kw(own(c).0, own(c).1) } else { [0.0; 2] }).collect();
            let mu_t3: Vec<f64> = (0..nc).map(|c| if fluid(c) { mut_of(own(c).0, own(c).1) } else { 0.0 }).collect();
            let mut case = Case::from_w(
                wedge(ni, nj, wall, deg.to_radians(), true, &f).g3,
                wd.w3.clone(),
                WEDGE_SYM,
                WEDGE_BC,
                &[],
                DT,
                turb3.clone(),
                Some(mu_t3),
                SST_RE,
            );
            // the strain of both updates is the one of the 2D cell gradient (`s.grad`, with the boundary ghosts of the 2D solver), written as a Cartesian tensor in the meridian
            // frame of the wedge cell: the comparison is then of the turbulence kernels, not of the two mean-flow gradients (`viscous3` parity is tested there)
            let (sn, co) = wd.theta_m.sin_cos();
            for c in 0..nc {
                if !fluid(c) {
                    continue;
                }
                let (gm, v, r) = (s.grad[c], s.w[crate::solver::pidx(ni, (c % ni) as isize, (c / ni) as isize)][2], s.grid.rc[c]);
                let (ex, er, et) = ([1.0, 0.0, 0.0], [0.0, sn, co], [0.0, co, -sn]);
                let mut g = [0.0; 12];
                for a in 0..3 {
                    for b in 0..3 {
                        g[3 * a + b] = gm[0] * ex[a] * ex[b] + gm[1] * ex[a] * er[b] + gm[2] * er[a] * ex[b] + gm[3] * er[a] * er[b] + v / r * et[a] * et[b];
                    }
                }
                case.grad[c] = g;
            }
            if !auto {
                for c in 0..nc {
                    if case.wall[c] != [0.0; 3] {
                        let nu = case.mu[c] / case.w[pidx3(ni, nj, 1, (c % ni) as isize, (c / ni) as isize, 0)][0];
                        case.wall[c] = [0.0, 60.0 * nu / (BETA1 * case.dwall[c] * case.dwall[c]), 0.0];
                    }
                }
            }
            let up = update(&case.input(), &Sst { re_c: SST_RE });
            // ---- compare
            let keep: Vec<bool> = (0..nc)
                .map(|c| {
                    let (i, j) = (c % ni, c / ni);
                    let axis = blk.is_none_or(|b| i >= b.i_end);
                    fluid(c) && i >= 2 && i + 2 < ni && j + 2 < nj && !(axis && j < 8)
                })
                .collect();
            let (mut d2, mut n2, mut dm, mut nm) = ([0.0f64; 3], [0.0f64; 3], [0.0f64; 3], [0.0f64; 3]);
            for c in (0..nc).filter(|&c| keep[c]) {
                let a = [s.turb[c][0] - turb0[c][0], s.turb[c][1] - turb0[c][1], s.mu_t[c]];
                let b = [up.turb[c][0] - turb3[c][0], up.turb[c][1] - turb3[c][1], up.mu_t[c]];
                for m in 0..3 {
                    d2[m] += (a[m] - b[m]).powi(2);
                    n2[m] += a[m] * a[m];
                    dm[m] = dm[m].max((a[m] - b[m]).abs());
                    nm[m] = nm[m].max(a[m].abs());
                }
            }
            ([0, 1, 2].map(|m| (d2[m] / n2[m]).sqrt()), [0, 1, 2].map(|m| dm[m] / nm[m]))
        }
    }

    /// Table of the measured differences (`--ignored --no-capture`).
    #[test]
    #[ignore = "prints the parity table"]
    fn parity_report() {
        for (wall, auto) in [(false, false), (true, false), (true, true)] {
            for (ni, nj) in [(48usize, 36usize), (96, 72)] {
                for deg in [6.0, 2.0, 1.0] {
                    let (l2, linf) = parity::run(ni, nj, wall, deg, auto);
                    println!(
                        "wall {wall:5} auto {auto:5} {ni}x{nj} {deg} deg: l2 [{:.2e} {:.2e} {:.2e}] linf [{:.2e} {:.2e} {:.2e}]",
                        l2[0], l2[1], l2[2], linf[0], linf[1], linf[2]
                    );
                }
            }
        }
    }

    /// One turbulence update of the 3D wedge against `turbulence::update` of the 2D solver on the same axisymmetric state, scalars, eddy viscosity, time step, mass
    /// flux and strain (relative `L2` difference over the inner cells of the change `dk`, `domega` of the scalars and of the new `mu_t`):
    ///
    /// * the difference does not depend on the opening angle (6, 2, 1 degrees give the same numbers to 3 digits): it is the difference of the two discretisations, not
    ///   a wedge effect (a ghost that follows its cell, the symmetry planes, adds nothing to the diagonal of the `k` factor: with the 2D rule it would be
    ///   `mu dt / (r dtheta)^2`, i.e. an `O(1 / dtheta^2)` damping of every update);
    /// * without a body `[1.4e-4 1.0e-4 1.5e-5]` (48x36) / `[2.6e-4 1.9e-4 2.6e-5]` (96x72); with a body (2D wall state `[0, 60 nu / (beta1 d^2), 0]`) `[4.7e-4 6.5e-3
    ///   1.4e-3]` / `[2.8e-4 1.6e-3 2.3e-4]`: the changes of `omega` and of `mu_t` fall by 4 and 6 with the grid (the first cells of the wall have a one-sided scalar
    ///   gradient: area-weighted in 3D, planar in 2D);
    /// * the automatic wall treatment gives the same numbers (the first cells of this state are at `y+ ~ 0.3`, where it is the 2D wall state: see
    ///   `wall_state_follows_the_sublayer_and_the_log_layer`).
    #[test]
    fn wedge_update_matches_the_2d_update() {
        let bound = |wall: bool, g: usize| match (wall, g) {
            (false, 0) => [3e-4, 3e-4, 5e-5],
            (false, _) => [5e-4, 4e-4, 6e-5],
            (true, 0) => [1e-3, 1.2e-2, 3e-3],
            (true, _) => [6e-4, 4e-3, 6e-4],
        };
        for (wall, auto) in [(false, false), (true, false), (true, true)] {
            let mut ref_l2 = [0.0; 3];
            for (g, (ni, nj)) in [(48usize, 36usize), (96, 72)].into_iter().enumerate() {
                for deg in [6.0, 1.0] {
                    let (l2, _) = parity::run(ni, nj, wall, deg, auto);
                    for m in 0..3 {
                        assert!(l2[m] <= bound(wall, g)[m], "wall {wall} auto {auto} {ni}x{nj} {deg} deg component {m}: {:e} > {:e}", l2[m], bound(wall, g)[m]);
                    }
                    if deg == 6.0 {
                        ref_l2 = l2;
                    } else {
                        // the same numbers at 1 degree
                        for m in 0..3 {
                            assert!((l2[m] / ref_l2[m] - 1.0).abs() < 0.02, "wall {wall} component {m}: {:e} at 6 deg, {:e} at 1 deg", ref_l2[m], l2[m]);
                        }
                    }
                }
            }
        }
    }

    // ---- cost ----------------------------------------------------------------------------------------------------------------------------------------

    /// Timing of the 3D kernels on a `100 x 100 x 50` = 0.5 M cell grid with two solid blocks (`--ignored --no-capture`).
    #[test]
    #[ignore = "timing"]
    fn timing_of_half_a_million_cells() {
        use std::time::Instant;
        let n = [100, 100, 50];
        let mut g = box_grid(n, [0.02, 0.02, 0.04]);
        set_solids(&mut g, vec![SolidBox { i: 20..80, j: 20..80, k: 0..24 }, SolidBox { i: 10..14, j: 10..90, k: 24..50 }]);
        let solid = solid_mask(&g);
        let nc = solid.len();
        let re_c = 1.0e6;
        let mut rng = Rng::new(1);
        let cells: Vec<Prim3> = (0..nc)
            .map(|c| {
                let x = g.xc[c];
                [1.0 + 0.1 * (3.0 * x[0]).sin(), 0.8 + 0.1 * (2.0 * x[1]).cos(), 0.05 * (x[2] * 5.0).sin(), 0.02 * rng.range(-1.0, 1.0), 0.7]
            })
            .collect();
        let w = padded(&g, &solid, &cells);
        let model = Sst { re_c };
        let t = Instant::now();
        let dirs = face_dirs(&g, &solid, &[false; 6]);
        let t_dirs = t.elapsed();
        let t = Instant::now();
        let dwall = wall_distance3(&g, &solid, &wall_faces(&g, &solid));
        let t_dist = t.elapsed();
        let (mut grad, mut mu) = (vec![[0.0; 12]; nc], vec![0.0; nc]);
        let t = Instant::now();
        cell_pass(&visc(re_c), &g, &solid, &w, &mut grad, &mut mu);
        let t_cell = t.elapsed();
        let mut wall = vec![[0.0; 3]; nc];
        let t = Instant::now();
        wall_state_pass(&g, &solid, &dirs, &w, &mu, &dwall, &[], false, &mut wall);
        let t_wall = t.elapsed();
        let mu_t = vec![1e-5; nc];
        let (mut fi, mut fj, mut fk) = (vec![[0.0; 5]; g.iface.len()], vec![[0.0; 5]; g.jface.len()], vec![[0.0; 5]; g.kface.len()]);
        let t = Instant::now();
        face_pass(&g, &solid, &w, &dirs, &grad, &mu, &mu_t, &wall, &mut fi, &mut fj, &mut fk);
        let t_face = t.elapsed();
        // mass flux for the update
        let f = [0, 1, 2].map(|dir| {
            let [n0, n1, n2] = face_shape(&g, dir);
            let mut v = vec![[0.0; 5]; n0 * n1 * n2];
            map_faces(&g, dir, &mut v, |i, j, k, _| match face_sides3(&g, &solid, &w, dir, i, j, k) {
                Some(s) => {
                    let n = face_of(&g, dir, i, j, k).1.n;
                    let mf = |w: &Prim3| w[0] * (w[1] * n[0] + w[2] * n[1] + w[3] * n[2]);
                    [0.5 * (mf(&s.l.w) + mf(&s.r.w)), 0.0, 0.0, 0.0, 0.0]
                }
                None => [0.0; 5],
            });
            v
        });
        let dt = vec![1e-3; nc];
        let turb = vec![model.inf(); nc];
        let cx = TurbIn {
            grid: &g,
            solid: &solid,
            w: &w,
            dirs: &dirs,
            bc: &ALL_INF,
            mu: &mu,
            mu_t: &mu_t,
            grad: &grad,
            wall: &wall,
            ks: &[],
            law_strain: false,
            law_i_walls: false,
            dwall: &dwall,
            fi: &f[0],
            fj: &f[1],
            fk: &f[2],
            dt: &dt,
            turb: &turb,
        };
        let t = Instant::now();
        let tg = scalar_gradients(&cx, &model);
        let t_grad = t.elapsed();
        let t = Instant::now();
        let blend = blends(&cx, &model, &tg);
        let t_blend = t.elapsed();
        let t = Instant::now();
        let tf = face_data(&cx, &model, &tg, &blend);
        let t_tface = t.elapsed();
        let t = Instant::now();
        let cells = cell_terms(&cx, &model, &tg, &blend, &tf);
        let t_terms = t.elapsed();
        let t = Instant::now();
        let up = solve_and_update(&cx, &model, &tg, &cells);
        let t_solve = t.elapsed();
        let t = Instant::now();
        let up2 = update(&cx, &model);
        let t_update = t.elapsed();
        assert_eq!(up.turb, up2.turb);
        println!("{nc} cells ({} fluid), {} threads", solid.iter().filter(|s| !**s).count(), rayon::current_num_threads());
        println!("viscous3: face_dirs (once per grid) {t_dirs:?}, cell_pass {t_cell:?}, face_pass {t_face:?}  (cell_pass + face_pass {:?})", t_cell + t_face);
        println!("walldist3 (once per grid) {t_dist:?}; wall_state_pass {t_wall:?}");
        println!("turbulence3 update {t_update:?} = gradients {t_grad:?} + blends {t_blend:?} + face data {t_tface:?} + cell terms {t_terms:?} + solves/update {t_solve:?}");
    }

    // ---- line solves and the scalar list ------------------------------------------------------------------------------------------------------------

    /// The line solves are exact tridiagonal solves along i, j and k, broken at solid cells: random coefficients (zero off-diagonals next to solids, as the kernel builds
    /// them) and a random right-hand side, then `A x = rhs` is checked line by line, for the two scalars at once; solid rows return 0.
    #[test]
    fn line_solves_are_exact_along_all_three_directions_with_solid_breaks() {
        let mut rng = Rng::new(9);
        let n = [6, 5, 4];
        let mut g = box_grid(n, [0.1; 3]);
        set_solids(&mut g, vec![SolidBox { i: 2..3, j: 1..4, k: 0..4 }, SolidBox { i: 0..6, j: 0..1, k: 2..3 }]);
        let solid = solid_mask(&g);
        let nc = solid.len();
        for dir in 0..3 {
            let stride = [1, n[0], n[0] * n[1]][dir];
            let len = n[dir];
            let pos = |c: usize| [c % n[0], (c / n[0]) % n[1], c / (n[0] * n[1])][dir];
            let mut fac = Factor::<2> { lo: vec![[0.0; 2]; nc], d: vec![[1.0; 2]; nc], up: vec![[0.0; 2]; nc] };
            for c in 0..nc {
                if solid[c] {
                    continue;
                }
                let lower = pos(c) > 0 && !solid[c - stride];
                let upper = pos(c) + 1 < len && !solid[c + stride];
                for e in 0..2 {
                    fac.lo[c][e] = if lower { -rng.range(0.1, 1.0) } else { 0.0 };
                    fac.up[c][e] = if upper { -rng.range(0.1, 1.0) } else { 0.0 };
                    fac.d[c][e] = 1.0 + rng.range(0.0, 2.0) - fac.lo[c][e] - fac.up[c][e];
                }
            }
            let rhs: Vec<[f64; 2]> = (0..nc).map(|c| if solid[c] { [0.0; 2] } else { [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)] }).collect();
            let mut x = rhs.clone();
            solve_lines(&g, &fac, dir, &mut x);
            for c in 0..nc {
                for e in 0..2 {
                    let mut ax = fac.d[c][e] * x[c][e];
                    if pos(c) > 0 {
                        ax += fac.lo[c][e] * x[c - stride][e];
                    }
                    if pos(c) + 1 < len {
                        ax += fac.up[c][e] * x[c + stride][e];
                    }
                    assert!((ax - rhs[c][e]).abs() < 1e-12, "dir {dir} cell {c} scalar {e}: A x = {ax} vs rhs {}", rhs[c][e]);
                }
                if solid[c] {
                    assert_eq!(x[c], [0.0; 2]);
                }
            }
        }
    }

    /// SST plus a passive transport scalar (a stand-in for a transition scalar): the list structure takes a third equation without touching the kernels, and `k`,
    /// `omega`, `mu_t` of the three-scalar update are those of the two-scalar one (the extra scalar does not couple back), while the third relaxes towards 1 and is
    /// convected and diffused.
    struct WithPassive(Sst);

    impl TurbModel<3> for WithPassive {
        fn inf(&self) -> [f64; 3] {
            let i = self.0.inf();
            [i[0], i[1], 0.0]
        }
        fn sigmas(&self, e: usize) -> [f64; 2] {
            if e < 2 {
                self.0.sigmas(e)
            } else {
                [1.0, 1.0]
            }
        }
        fn blend(&self, c: &CellIn<3>) -> Blend {
            self.0.blend(&two(c))
        }
        fn terms(&self, c: &CellIn<3>, b: &Blend) -> ([f64; 3], [f64; 3]) {
            let (s, d) = self.0.terms(&two(c), b);
            ([s[0], s[1], c.rho * 2.0], [d[0], d[1], 2.0])
        }
        fn wall_value(&self, e: usize, wall: &[f64; 3]) -> Option<f64> {
            if e < 2 {
                self.0.wall_value(e, wall)
            } else {
                None
            }
        }
        fn wall_cell(&self, e: usize, solved: f64, y_plus: f64, u_tau: f64, nu: f64) -> f64 {
            if e < 2 {
                self.0.wall_cell(e, solved, y_plus, u_tau, nu)
            } else {
                solved
            }
        }
        fn clip(&self, e: usize, old: f64, new: f64) -> f64 {
            if e < 2 {
                self.0.clip(e, old, new)
            } else {
                new
            }
        }
        fn eddy_viscosity(&self, c: &CellIn<3>) -> f64 {
            self.0.eddy_viscosity(&two(c))
        }
    }

    fn two(c: &CellIn<3>) -> CellIn<2> {
        CellIn { rho: c.rho, nu: c.nu, d: c.d, phi: [c.phi[0], c.phi[1]], grad: [c.grad[0], c.grad[1]], s2: c.s2, mu_t: c.mu_t }
    }

    #[test]
    fn a_third_scalar_is_an_addition() {
        let mut rng = Rng::new(21);
        let (nodes, n, cells, turb, ksv) = random_case(&mut rng, 3e-4);
        let case = Case::new(
            grid_with_solid(&nodes, n),
            &cells,
            [false; 6],
            [ScalarBc::Inf, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Cell],
            &ksv,
            0.2,
            turb.clone(),
            SST_RE,
        );
        let model2 = Sst { re_c: SST_RE };
        let model3 = WithPassive(model2);
        let a = update(&case.input(), &model2);
        let turb3: Vec<[f64; 3]> = turb.iter().enumerate().map(|(c, t)| [t[0], t[1], if case.solid[c] { 0.0 } else { 0.3 }]).collect();
        let cx3 = TurbIn {
            grid: &case.grid,
            solid: &case.solid,
            w: &case.w,
            dirs: &case.dirs,
            bc: &case.bc,
            mu: &case.mu,
            mu_t: &case.mu_t,
            grad: &case.grad,
            wall: &case.wall,
            ks: &[],
            law_strain: false,
            law_i_walls: false,
            dwall: &case.dwall,
            fi: &case.f[0],
            fj: &case.f[1],
            fk: &case.f[2],
            dt: &case.dt,
            turb: &turb3,
        };
        let b = update(&cx3, &model3);
        for c in 0..case.solid.len() {
            if case.solid[c] {
                continue;
            }
            for e in 0..2 {
                assert_eq!(a.turb[c][e], b.turb[c][e], "cell {c} scalar {e} is independent of the third equation");
            }
            assert_eq!(a.mu_t[c], b.mu_t[c]);
            assert!(
                b.turb[c][2].is_finite() && b.turb[c][2] > 0.0,
                "the passive scalar relaxes from 0.3 towards 1 (and is advected from the far field, 0): {}",
                b.turb[c][2]
            );
        }
    }
}
