//! Weiss-Smith low-Mach preconditioning of the 3D implicit left-hand side: the 5x5 counterpart of the 4x4 pieces of [`crate::implicit`] (`LowMach`,
//! `precond_matrix`, `precond_inverse`, `precond_speeds`, `spectral_pc`, `abs_jacobian_pc`), in the variables and layout of [`crate::implicit3`]
//! (`Prim3 = [rho, u, v, w, p]`, `Cons3 = [rho, rho u, rho v, rho w, E]`, row-major `M5`, the three Cartesian velocity components).
//!
//! The left-hand side `I/dt - dR/dU` becomes `P/dt - dR/dU` in all three factors, the defect operator and between the sweeps; the right-hand side is untouched, so the
//! converged state is that of the unpreconditioned scheme and only the pseudo-time dynamics at low Mach number change (2D: Mach 0.1 14 100 -> 1 200 iterations,
//! ISSUES #19). With `U_r` the reference velocity (`LowMach::u_ref`: `|u|` clamped to `[ur_min, c]`), `b = U_r^2 / c^2`, `a = (1, u, v, w, H)`, `g = (gamma - 1)
//! (q^2/2, -u, -v, -w, 1)` (`g.a = c^2`, `dp = g.dU`):
//!
//! ```text
//!   P      = I + (1 / U_r^2 - 1 / c^2) a g^T                      P^-1 = I - kappa/(1 + kappa c^2) a g^T,   kappa = 1/U_r^2 - 1/c^2        (= I for U_r = c)
//!   P^-1 A has the eigenvalues u_n (3 times), u' + c', u' - c':   u' = u_n (1 + b) / 2,   c' = sqrt(u_n^2 (1 - b)^2 + 4 b c^2) / 2          (= u_n, u_n +- c for b = 1)
//!   dissipation of a face:  P |P^-1 A|,  |P^-1 A| the quadratic polynomial through the entropy-fixed |lambda| at the three distinct eigenvalues (Roe average)
//!   pseudo-time step:       lambda = |u'| + c'  instead of |u_n| + c
//! ```
//!
//! The block scheme consumes `P` (`LowMach::p_matrix`: the diagonal block starts from `P` instead of `I`; `(P - dt J_i) X = dt R`, `(P - dt J_j) dU = P X`, ...,
//! the defect residual uses `P dU`), `abs_jacobian_pc3` (the face dissipation, with the shock-sensor blend towards the plain Rusanov `lambda I` of the right-hand side) and
//! `spectral_pc3` (the time step), exactly as `implicit::cell_blocks`, `step` and `compute_dt` do in 2D.
//!
//! # The diagonalised line solve (`LineSolve3::DiagonalCorrected`) with preconditioning: closed form, provided
//!
//! `diag3` replaces the factor `I - dt J_d` of a cell by `T (I + dt delta_d Lambda) T^-1` (the eigensystem of the flux Jacobian `A` at the cell normal). With
//! preconditioning the factor is `P + dt delta_d A = P (I + dt delta_d P^-1 A)`, so what it needs per cell is the eigensystem of `P^-1 A`: closed form,
//! [`eig_pc5`]. In primitive variables `W = (rho, u_n, u_t1, u_t2, p)` the preconditioned system is `Gamma W_tau + A_W W_n = 0` with `Gamma = [[1,0,kappa],[0,I,0],
//! [0,0,1 + kappa c^2]]` on `(rho, p)` (this is `P = M Gamma M^-1`, `M = dU/dW`), and its eigenvectors are
//!
//! ```text
//!   entropy  (u_n)       r = (1, 0, 0, 0, 0)            l = (1, 0, 0, 0, -1/c^2)
//!   tangents (u_n)       r = (0, t_k, 0)                l = (0, t_k, 0)                                      k = 1, 2
//!   acoustic (u' +- c')  r = (rho s / c^2, n, rho s)    l = +-(0, (-+ s_bar) n / (2c'), +- 1/(2 rho c'))      s = lambda - u_n = u' - u_n +- c', s_bar the other one
//! ```
//!
//! (`R = M R_W`, `L = L_W M^-1` in conservative variables). Then `(P + t A)^-1 = R (I + t Lambda)^-1 L P^-1`: the first sweep of the diagonal scheme applies `L P^-1`
//! ([`PcEig::l_pinv`]), the next sweeps `L` only (the `P X` between sweeps cancels the `P^-1`). The scalar coefficients are the same as without preconditioning with the
//! eigenvalues replaced: `lambda(A) -> lam_pc5` ([`lam_pc5`]) for the flux Jacobians and `|lambda'|` ([`roe_eigs_pc5`], [`face_dis_pc5`]) for the face dissipation. The one
//! approximation: the shock-sensor blend `t lambda I` of the dissipation is `t lambda P^-1` after multiplying by `P^-1`, which is not diagonal in the acoustic
//! eigenvectors; `face_dis_pc5` keeps it as the scalar `t lambda` (the defect-correction pass against the block operator, which has `P` exactly, absorbs it, as it absorbs the
//! frozen eigenvectors of the 3D diagonal scheme). Tangent vectors: `tangents` is the rule of `diag3` (duplicated: the module is private to `implicit3`).
//!
//! Not provided (belongs to the solver): where `P` enters the boundary treatment (the ghost Jacobian blocks are unchanged), and the `ImplicitParams` flag
//! (`low_mach_precond`, documented as 2D only).

use crate::gas3::{sound_speed3, Cons3, Prim3, GAMMA};
use crate::implicit::{ImplicitParams, LOW_MACH_UINF_CUTOFF, LOW_MACH_UREF_EPS};
use crate::implicit3::{abs_jacobian3, flux_jacobian3, M5};
use crate::limiter::shock_weight;

const EYE: M5 = {
    let mut m = [0.0; 25];
    let mut q = 0;
    while q < 5 {
        m[6 * q] = 1.0;
        q += 1;
    }
    m
};

#[inline]
fn mm(a: &M5, b: &M5) -> M5 {
    let mut c = [0.0; 25];
    for i in 0..5 {
        for k in 0..5 {
            let aik = a[5 * i + k];
            for j in 0..5 {
                c[5 * i + j] += aik * b[5 * k + j];
            }
        }
    }
    c
}

/// Low-Mach preconditioning of the implicit left-hand side.
#[derive(Clone, Copy, Debug)]
pub struct LowMach {
    /// `max(LOW_MACH_UREF_EPS c_inf, LOW_MACH_UINF_CUTOFF |u_inf|)`: lower bound of the reference velocity.
    pub ur_min: f64,
}

impl LowMach {
    /// The preconditioning of a free stream `fs` when `p.low_mach_precond` is set.
    pub fn of(p: &ImplicitParams, fs: &Prim3) -> Option<LowMach> {
        p.low_mach_precond.then(|| LowMach::for_freestream(fs))
    }

    /// `ur_min` of the free stream `fs`: `eps c_inf`, cut off at `|u_inf|` (so `P = I` in the free stream of a supersonic run).
    pub fn for_freestream(fs: &Prim3) -> LowMach {
        LowMach { ur_min: (LOW_MACH_UREF_EPS * sound_speed3(fs)).max(LOW_MACH_UINF_CUTOFF * (fs[1] * fs[1] + fs[2] * fs[2] + fs[3] * fs[3]).sqrt()) }
    }

    /// Reference velocity `U_r = clamp(|u|, ur_min, c)` of the state `w` with sound speed `c`.
    #[inline]
    pub fn u_ref(&self, w: &Prim3, c: f64) -> f64 {
        (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]).sqrt().max(self.ur_min).min(c)
    }

    /// `P` of the state `w` (the identity when `U_r = c`).
    pub fn p_matrix(&self, w: &Prim3) -> M5 {
        let c = sound_speed3(w);
        let ur = self.u_ref(w, c);
        if ur >= c {
            return EYE;
        }
        precond_matrix5(w, ur)
    }
}

/// `a`, `g` of `P = I + kappa a g^T` and `kappa`, `c^2`.
#[inline]
fn p_parts(w: &Prim3, ur: f64) -> ([f64; 5], [f64; 5], f64, f64) {
    let [rho, u, v, ww, p] = *w;
    let g1 = GAMMA - 1.0;
    let c2 = GAMMA * p / rho;
    let q2 = u * u + v * v + ww * ww;
    let h = GAMMA * p / (g1 * rho) + 0.5 * q2;
    ([1.0, u, v, ww, h], [g1 * 0.5 * q2, -g1 * u, -g1 * v, -g1 * ww, g1], 1.0 / (ur * ur) - 1.0 / c2, c2)
}

/// `P = I + (1/U_r^2 - 1/c^2) a g^T` for the reference velocity `ur`.
pub fn precond_matrix5(w: &Prim3, ur: f64) -> M5 {
    let (a, g, kappa, _) = p_parts(w, ur);
    let mut m = EYE;
    for i in 0..5 {
        for j in 0..5 {
            m[5 * i + j] += kappa * a[i] * g[j];
        }
    }
    m
}

/// `P^-1 = I - kappa a g^T / (1 + kappa c^2)` (`g . a = c^2`).
pub fn precond_inverse5(w: &Prim3, ur: f64) -> M5 {
    let (a, g, kappa, c2) = p_parts(w, ur);
    let f = kappa / (1.0 + kappa * c2);
    let mut m = EYE;
    for i in 0..5 {
        for j in 0..5 {
            m[5 * i + j] -= f * a[i] * g[j];
        }
    }
    m
}

/// `P x` of a conserved-variable vector (the multiplication between the sweeps and in the defect residual), without forming `P`.
#[inline]
pub fn p_apply(w: &Prim3, ur: f64, x: &Cons3) -> Cons3 {
    let (a, g, kappa, _) = p_parts(w, ur);
    let s = kappa * (g[0] * x[0] + g[1] * x[1] + g[2] * x[2] + g[3] * x[3] + g[4] * x[4]);
    [x[0] + s * a[0], x[1] + s * a[1], x[2] + s * a[2], x[3] + s * a[3], x[4] + s * a[4]]
}

/// Preconditioned eigenvalue data of the normal speed `un` (any sign), `b = U_r^2 / c^2`: `(u', c')` with `u' = un (1 + b)/2`,
/// `c' = sqrt(un^2 (1-b)^2 + 4 b c^2)/2`; the largest eigenvalue magnitude is `|u'| + c'`, `= |un| + c` for `b = 1`.
#[inline]
pub fn precond_speeds(un: f64, c: f64, ur: f64) -> (f64, f64) {
    let b = ur * ur / (c * c);
    (0.5 * un * (1.0 + b), 0.5 * (un * un * (1.0 - b) * (1.0 - b) + 4.0 * b * c * c).sqrt())
}

/// Preconditioned spectral radius `|u'| + c'` of the state `w` for the unit normal `n`.
#[inline]
pub fn spectral_pc3(w: &Prim3, n: [f64; 3], lm: &LowMach) -> f64 {
    let c = sound_speed3(w);
    let un = w[1] * n[0] + w[2] * n[1] + w[3] * n[2];
    let (u1, c1) = precond_speeds(un, c, lm.u_ref(w, c));
    u1.abs() + c1
}

/// The eigenvalues of `P^-1 A(w, n)`: `u_n` (three times), `u' + c'`, `u' - c'`.
pub fn lam_pc5(w: &Prim3, n: [f64; 3], lm: &LowMach) -> [f64; 5] {
    let c = sound_speed3(w);
    let un = w[1] * n[0] + w[2] * n[1] + w[3] * n[2];
    let (u1, c1) = precond_speeds(un, c, lm.u_ref(w, c));
    [un, un, un, u1 + c1, u1 - c1]
}

/// Roe average of two states: `(u, v, w, H, c)`.
fn roe(wl: &Prim3, wr: &Prim3) -> ([f64; 3], f64, f64) {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0 / (sl + sr);
    let vel = [(sl * wl[1] + sr * wr[1]) * inv, (sl * wl[2] + sr * wr[2]) * inv, (sl * wl[3] + sr * wr[3]) * inv];
    let ke = |w: &Prim3| 0.5 * (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]);
    let hl = GAMMA * wl[4] / ((GAMMA - 1.0) * wl[0]) + ke(wl);
    let hr = GAMMA * wr[4] / ((GAMMA - 1.0) * wr[0]) + ke(wr);
    let h = (sl * hl + sr * hr) * inv;
    let c = ((GAMMA - 1.0) * (h - 0.5 * (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]))).max(1e-300).sqrt();
    (vel, h, c)
}

/// The entropy-fixed `|lambda|` (`eps_fix c` floor, quadratic blend) of the three distinct eigenvalues `(u_n, u' + c', u' - c')` of `P^-1 A` at the Roe average of
/// `wl`, `wr`: with `b = 1` (no preconditioning) the `(|u_n|, |u_n + c|, |u_n - c|)` of `diag3`.
pub fn roe_eigs_pc5(wl: &Prim3, wr: &Prim3, n: [f64; 3], eps_fix: f64, lm: &LowMach) -> (f64, f64, f64) {
    let (u, _, c) = roe(wl, wr);
    let a = u[0] * n[0] + u[1] * n[1] + u[2] * n[2];
    let q = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    let ur = q.sqrt().max(lm.ur_min).min(c);
    let (u1, c1) = precond_speeds(a, c, ur);
    let eps = eps_fix * c;
    let fix = |l: f64| if l.abs() >= eps { l.abs() } else { 0.5 * (l * l / eps + eps) };
    (fix(a), fix(u1 + c1), fix(u1 - c1))
}

/// Dissipation eigenvalues of the face between `wa` and `wb` for the diagonal scheme: the Roe-type `|P^-1 A|` eigenvalues of [`roe_eigs_pc5`] blended by the shock sensor
/// with the (unpreconditioned) Rusanov `lambda` of the right-hand side, as `diag3::face_dis3` (see the module documentation for the approximation in the blend).
pub fn face_dis_pc5(wa: &Prim3, wb: &Prim3, n: [f64; 3], p: &ImplicitParams, lm: &LowMach) -> [f64; 5] {
    let sp = |w: &Prim3| (w[1] * n[0] + w[2] * n[1] + w[3] * n[2]).abs() + sound_speed3(w);
    let lamf = sp(wa).max(sp(wb));
    let (e1, e2, e3) = if p.roe {
        let (e1, e2, e3) = roe_eigs_pc5(wa, wb, n, p.eps_fix, lm);
        let t = shock_weight(wa[4], wb[4]);
        ((1.0 - t) * e1 + t * lamf, (1.0 - t) * e2 + t * lamf, (1.0 - t) * e3 + t * lamf)
    } else {
        (lamf, lamf, lamf)
    };
    [e1, e1, e1, e2, e3]
}

/// The preconditioned dissipation matrix `P |P^-1 A|` of the face between `wl` and `wr` (Roe average): `|M|` for `M = P^-1 A` (eigenvalues `u_n` three times and
/// `u' +- c'`) is the quadratic polynomial through the entropy-fixed `|lambda|` at the three distinct eigenvalues (Newton form), the result is multiplied by `P`. The
/// plain `abs_jacobian3` when `U_r >= c` at the Roe state.
pub fn abs_jacobian_pc3(wl: &Prim3, wr: &Prim3, n: [f64; 3], eps_fix: f64, lm: &LowMach) -> M5 {
    let (u, h, c) = roe(wl, wr);
    let q2h = 0.5 * (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    let ur = (2.0 * q2h).sqrt().max(lm.ur_min).min(c);
    if ur >= c {
        return abs_jacobian3(wl, wr, n, eps_fix);
    }
    let a = u[0] * n[0] + u[1] * n[1] + u[2] * n[2];
    // the Roe state with rho = 1: A and P depend on (u, H) only
    let wa: Prim3 = [1.0, u[0], u[1], u[2], (GAMMA - 1.0) / GAMMA * (h - q2h)];
    let (am, p, pi) = (flux_jacobian3(&wa, n), precond_matrix5(&wa, ur), precond_inverse5(&wa, ur));
    let m = mm(&pi, &am);
    let (u1, c1) = precond_speeds(a, c, ur);
    let (l2, l3) = (u1 + c1, u1 - c1);
    let eps = eps_fix * c;
    let fix = |l: f64| if l.abs() >= eps { l.abs() } else { 0.5 * (l * l / eps + eps) };
    let (e1, e2, e3) = (fix(a), fix(l2), fix(l3));
    let d1 = (e2 - e1) / (l2 - a);
    let d2 = ((e3 - e1) / (l3 - a) - d1) / (l3 - l2);
    let (mut b1, mut b2) = (m, m);
    for q in 0..5 {
        b1[6 * q] -= a;
        b2[6 * q] -= l2;
    }
    let prod = mm(&b1, &b2);
    let mut pm = [0.0; 25];
    for k in 0..25 {
        pm[k] = d1 * b1[k] + d2 * prod[k];
    }
    for q in 0..5 {
        pm[6 * q] += e1;
    }
    mm(&p, &pm)
}

/// Two unit tangents orthogonal to the unit normal `n` (and to each other): the rule of `implicit3::diag3` (lowest axis within `TIE_TOL`), which the GPU solver shares.
pub fn tangents(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    const TIE_TOL: f64 = 1e-4;
    let a = if n[0].abs() <= n[1].abs() + TIE_TOL && n[0].abs() <= n[2].abs() + TIE_TOL {
        0
    } else if n[1].abs() <= n[2].abs() + TIE_TOL {
        1
    } else {
        2
    };
    let mut e = [0.0; 3];
    e[a] = 1.0;
    let mut t1 = [e[1] * n[2] - e[2] * n[1], e[2] * n[0] - e[0] * n[2], e[0] * n[1] - e[1] * n[0]];
    let l = (t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]).sqrt();
    for x in t1.iter_mut() {
        *x /= l;
    }
    let t2 = [n[1] * t1[2] - n[2] * t1[1], n[2] * t1[0] - n[0] * t1[2], n[0] * t1[1] - n[1] * t1[0]];
    (t1, t2)
}

/// The closed-form eigensystem of `P^-1 A(w, n)`: `P^-1 A = R diag(lam) L`, `L R = I`, `l_pinv = L P^-1`.
#[derive(Clone, Copy, Debug)]
pub struct PcEig {
    pub lam: [f64; 5],
    /// Right eigenvectors as columns, `r[row][field]`.
    pub r: [[f64; 5]; 5],
    /// Left eigenvectors as rows, `l[field][col]`.
    pub l: [[f64; 5]; 5],
    /// `L P^-1`: the left factor of the first sweep, `(P + t A)^-1 = R (I + t diag(lam))^-1 L P^-1`.
    pub l_pinv: [[f64; 5]; 5],
}

/// The eigensystem of `P^-1 A(w, n)` for the reference velocity `ur` (see the module documentation for the primitive-variable eigenvectors; conservative: `R = M R_W`,
/// `L = L_W M^-1`, `L P^-1 = L_W Gamma^-1 M^-1`, `M = dU/dW`). The fields are ordered `(entropy, t1, t2, u' + c', u' - c')`.
pub fn eig_pc5(w: &Prim3, n: [f64; 3], ur: f64) -> PcEig {
    let [rho, u, v, ww, p] = *w;
    let g1 = GAMMA - 1.0;
    let c2 = GAMMA * p / rho;
    let c = c2.sqrt();
    let un = u * n[0] + v * n[1] + ww * n[2];
    let b = ur * ur / c2;
    let (u1, c1) = precond_speeds(un, c, ur);
    let (sp, sm) = (u1 + c1 - un, u1 - c1 - un);
    let (t1, t2) = tangents(n);
    let q2 = u * u + v * v + ww * ww;
    // M = dU/dW and M^-1 = dW/dU
    let m: [[f64; 5]; 5] = [
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [u, rho, 0.0, 0.0, 0.0],
        [v, 0.0, rho, 0.0, 0.0],
        [ww, 0.0, 0.0, rho, 0.0],
        [0.5 * q2, rho * u, rho * v, rho * ww, 1.0 / g1],
    ];
    let mi: [[f64; 5]; 5] = [
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [-u / rho, 1.0 / rho, 0.0, 0.0, 0.0],
        [-v / rho, 0.0, 1.0 / rho, 0.0, 0.0],
        [-ww / rho, 0.0, 0.0, 1.0 / rho, 0.0],
        [0.5 * g1 * q2, -g1 * u, -g1 * v, -g1 * ww, g1],
    ];
    // R_W (columns) and L_W (rows) in primitive variables
    let rw: [[f64; 5]; 5] = [
        [1.0, 0.0, 0.0, rho * sp / c2, rho * sm / c2],
        [0.0, t1[0], t2[0], n[0], n[0]],
        [0.0, t1[1], t2[1], n[1], n[1]],
        [0.0, t1[2], t2[2], n[2], n[2]],
        [0.0, 0.0, 0.0, rho * sp, rho * sm],
    ];
    let (a_p, a_m) = (-sm / (2.0 * c1), sp / (2.0 * c1));
    let (b_p, b_m) = (1.0 / (2.0 * rho * c1), -1.0 / (2.0 * rho * c1));
    let lw: [[f64; 5]; 5] = [
        [1.0, 0.0, 0.0, 0.0, -1.0 / c2],
        [0.0, t1[0], t1[1], t1[2], 0.0],
        [0.0, t2[0], t2[1], t2[2], 0.0],
        [0.0, a_p * n[0], a_p * n[1], a_p * n[2], b_p],
        [0.0, a_m * n[0], a_m * n[1], a_m * n[2], b_m],
    ];
    // Gamma^-1 on (rho, p): [[1, -(1 - b)/c^2], [0, b]]
    let mut gi = [[0.0; 5]; 5];
    for (q, row) in gi.iter_mut().enumerate() {
        row[q] = 1.0;
    }
    gi[0][4] = -(1.0 - b) / c2;
    gi[4][4] = b;
    let (mut r, mut l, mut l_pinv) = ([[0.0; 5]; 5], [[0.0; 5]; 5], [[0.0; 5]; 5]);
    for i in 0..5 {
        for j in 0..5 {
            for k in 0..5 {
                r[i][j] += m[i][k] * rw[k][j];
                l[i][j] += lw[i][k] * mi[k][j];
            }
        }
    }
    let mut lg = [[0.0; 5]; 5];
    for i in 0..5 {
        for j in 0..5 {
            for k in 0..5 {
                lg[i][j] += lw[i][k] * gi[k][j];
            }
        }
    }
    for i in 0..5 {
        for j in 0..5 {
            for k in 0..5 {
                l_pinv[i][j] += lg[i][k] * mi[k][j];
            }
        }
    }
    PcEig { lam: [un, un, un, u1 + c1, u1 - c1], r, l, l_pinv }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux3::tests::{random_rotation, rot, Rng};
    use crate::implicit::{abs_jacobian_pc, precond_inverse, precond_matrix, LowMach as LowMach2};
    use crate::implicit3::inv5;

    fn unit(rng: &mut Rng) -> [f64; 3] {
        let n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        [n[0] / l, n[1] / l, n[2] / l]
    }

    fn state(rng: &mut Rng, vmax: f64) -> Prim3 {
        [rng.range(0.6, 1.6), rng.range(-vmax, vmax), rng.range(-vmax, vmax), rng.range(-vmax, vmax), rng.range(0.6, 1.4)]
    }

    fn max_abs(a: &M5, b: &M5) -> f64 {
        a.iter().zip(b).fold(0.0f64, |m, (x, y)| m.max((x - y).abs()))
    }

    fn to_m5(m: &[[f64; 5]; 5]) -> M5 {
        let mut o = [0.0; 25];
        for i in 0..5 {
            for j in 0..5 {
                o[5 * i + j] = m[i][j];
            }
        }
        o
    }

    /// `P P^-1 = I`, `P = I` at `U_r = c`, `P x` matches the matrix.
    #[test]
    fn precond_matrix_and_inverse() {
        let mut rng = Rng::new(1);
        for _ in 0..200 {
            let w = state(&mut rng, 0.8);
            let c = sound_speed3(&w);
            let ur = rng.range(0.02, 1.0) * c;
            let (p, pi) = (precond_matrix5(&w, ur), precond_inverse5(&w, ur));
            assert!(max_abs(&mm(&p, &pi), &EYE) < 1e-12);
            assert!(max_abs(&precond_matrix5(&w, c), &EYE) < 1e-14);
            let x: Cons3 = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
            let px = p_apply(&w, ur, &x);
            for i in 0..5 {
                let want: f64 = (0..5).map(|j| p[5 * i + j] * x[j]).sum();
                assert!((px[i] - want).abs() < 1e-12 * (1.0 + want.abs()));
            }
        }
    }

    /// The 5x5 `P`, `P^-1`, `|A|_pc` of a state with no `y` velocity and a normal in the `x z` plane are the 4x4 ones of `implicit.rs` with `v = w`: the entries of the
    /// rows / columns `(rho, rho u, rho w, E)` agree, the `y`-momentum row / column is decoupled (`P`: identity there; dissipation: `|u_n|`-like diagonal).
    #[test]
    fn reduces_to_the_2d_four_by_four() {
        let idx = [0usize, 1, 3, 4];
        let mut rng = Rng::new(2);
        let lm3 = LowMach { ur_min: 0.1 };
        let lm2 = LowMach2 { ur_min: 0.1 };
        for it in 0..300 {
            let (rho, u, ww, p) = (rng.range(0.6, 1.6), rng.range(-0.6, 0.9), rng.range(-0.6, 0.6), rng.range(0.6, 1.4));
            let w3: Prim3 = [rho, u, 0.0, ww, p];
            let w2 = [rho, u, ww, p];
            let c = sound_speed3(&w3);
            let ur = lm3.u_ref(&w3, c);
            let (p5, pi5) = (precond_matrix5(&w3, ur), precond_inverse5(&w3, ur));
            let (p4, pi4) = (precond_matrix(&w2, ur), precond_inverse(&w2, ur));
            for (a, &ia) in idx.iter().enumerate() {
                for (b, &ib) in idx.iter().enumerate() {
                    assert!((p5[5 * ia + ib] - p4[4 * a + b]).abs() < 1e-13 && (pi5[5 * ia + ib] - pi4[4 * a + b]).abs() < 1e-13, "it {it}");
                }
            }
            // y momentum untouched
            for k in 0..5 {
                assert!((p5[10 + k] - if k == 2 { 1.0 } else { 0.0 }).abs() < 1e-14 && (p5[5 * k + 2] - if k == 2 { 1.0 } else { 0.0 }).abs() < 1e-14);
            }
            // dissipation of two neighbouring states
            let wr3: Prim3 = [rho * rng.range(0.9, 1.1), u * rng.range(0.9, 1.1), 0.0, ww * rng.range(0.9, 1.1), p * rng.range(0.9, 1.1)];
            let wr2 = [wr3[0], wr3[1], wr3[3], wr3[4]];
            let th = rng.range(0.0, std::f64::consts::TAU);
            let (nx, nz) = (th.cos(), th.sin());
            let d5 = abs_jacobian_pc3(&w3, &wr3, [nx, 0.0, nz], 0.1, &lm3);
            let d4 = abs_jacobian_pc(&w2, &wr2, nx, nz, 0.1, &lm2);
            for (a, &ia) in idx.iter().enumerate() {
                for (b, &ib) in idx.iter().enumerate() {
                    assert!(
                        (d5[5 * ia + ib] - d4[4 * a + b]).abs() < 1e-12 * (1.0 + d4[4 * a + b].abs()),
                        "it {it}: |A|_pc [{a}][{b}] {} vs {}",
                        d5[5 * ia + ib],
                        d4[4 * a + b]
                    );
                }
            }
        }
    }

    /// Without preconditioning (`ur_min >= c`) the dissipation is `abs_jacobian3`.
    #[test]
    fn no_preconditioning_when_the_reference_velocity_reaches_the_sound_speed() {
        let mut rng = Rng::new(3);
        let lm = LowMach { ur_min: 10.0 };
        for _ in 0..50 {
            let (wl, wr, n) = (state(&mut rng, 0.5), state(&mut rng, 0.5), unit(&mut rng));
            assert!(max_abs(&abs_jacobian_pc3(&wl, &wr, n, 0.1, &lm), &abs_jacobian3(&wl, &wr, n, 0.1)) == 0.0);
            assert!(max_abs(&lm.p_matrix(&wl), &EYE) == 0.0);
            let e = eig_pc5(&wl, n, sound_speed3(&wl));
            let lam = lam_pc5(&wl, n, &lm);
            let c = sound_speed3(&wl);
            let un = wl[1] * n[0] + wl[2] * n[1] + wl[3] * n[2];
            assert!((lam[3] - (un + c)).abs() < 1e-13 && (lam[4] - (un - c)).abs() < 1e-13 && (e.lam[3] - lam[3]).abs() < 1e-13);
            assert!((spectral_pc3(&wl, n, &lm) - (un.abs() + c)).abs() < 1e-13);
        }
    }

    /// Coefficients `c_k` of the characteristic polynomial `det(lambda I - M) = sum c_k lambda^(5-k)` by the Faddeev-LeVerrier recursion.
    fn charpoly(m: &M5) -> [f64; 6] {
        let mut c = [0.0; 6];
        c[0] = 1.0;
        let mut mk = EYE;
        for k in 1..=5 {
            let am = mm(m, &mk);
            let tr: f64 = (0..5).map(|i| am[6 * i]).sum();
            c[k] = -tr / k as f64;
            mk = am;
            for i in 0..5 {
                mk[6 * i] += c[k];
            }
        }
        c
    }

    /// The preconditioned wave speeds against the eigenvalues of `P^-1 A` computed numerically: the characteristic polynomial of the matrix `P^-1 A` (Faddeev-LeVerrier on
    /// the 5x5 matrix) equals `(lambda - u_n)^3 (lambda - u' - c') (lambda - u' + c')` coefficient by coefficient, i.e. the eigenvalues are `u_n` (3 times) and `u' +- c'`
    /// of `precond_speeds`; the spectral radius is `|u'| + c'` and never above `|u_n| + c`; and the eigensystem is exact: `M R = R Lambda`, `L R = I`.
    #[test]
    fn wave_speeds_are_the_eigenvalues_of_p_inverse_a() {
        let mut rng = Rng::new(4);
        let lm = LowMach { ur_min: 0.05 };
        for it in 0..300 {
            let w = state(&mut rng, [0.05, 0.4, 1.5][it % 3]);
            let n = unit(&mut rng);
            let c = sound_speed3(&w);
            let ur = lm.u_ref(&w, c);
            let m = mm(&precond_inverse5(&w, ur), &flux_jacobian3(&w, n));
            let lam = lam_pc5(&w, n, &lm);
            // polynomial coefficients from the predicted eigenvalues
            let mut poly = [0.0; 6];
            poly[0] = 1.0;
            for (k, &l) in lam.iter().enumerate() {
                for j in (1..=k + 1).rev() {
                    poly[j] -= l * poly[j - 1];
                }
            }
            let cp = charpoly(&m);
            let scale = lam.iter().fold(1.0f64, |s, l| s.max(l.abs()));
            for k in 0..6 {
                assert!((cp[k] - poly[k]).abs() < 1e-11 * scale.powi(k as i32), "it {it}: c_{k} {} vs {}", cp[k], poly[k]);
            }
            let un = w[1] * n[0] + w[2] * n[1] + w[3] * n[2];
            assert!(spectral_pc3(&w, n, &lm) <= un.abs() + c + 1e-13);
            assert!((spectral_pc3(&w, n, &lm) - lam.iter().fold(0.0f64, |s, l| s.max(l.abs()))).abs() < 1e-13);
            // eigensystem
            let e = eig_pc5(&w, n, ur);
            let (r, l) = (to_m5(&e.r), to_m5(&e.l));
            assert!(max_abs(&mm(&l, &r), &EYE) < 1e-10, "it {it}");
            let mr = mm(&m, &r);
            let mut rl = r;
            for i in 0..5 {
                for j in 0..5 {
                    rl[5 * i + j] *= e.lam[j];
                }
            }
            assert!(max_abs(&mr, &rl) < 1e-10 * scale, "it {it}: M R vs R Lambda {}", max_abs(&mr, &rl));
            assert!(max_abs(&to_m5(&e.l_pinv), &mm(&l, &precond_inverse5(&w, ur))) < 1e-10, "it {it}: L P^-1");
        }
    }

    /// `(P + t A)^-1 = R (I + t Lambda)^-1 L P^-1`: the closed-form diagonalised inverse against the dense inverse, for several `t` of either sign.
    #[test]
    fn diagonalised_inverse_of_p_plus_t_a() {
        let mut rng = Rng::new(5);
        let lm = LowMach { ur_min: 0.1 };
        for _ in 0..100 {
            let w = state(&mut rng, 0.3);
            let n = unit(&mut rng);
            let c = sound_speed3(&w);
            let ur = lm.u_ref(&w, c);
            let e = eig_pc5(&w, n, ur);
            let (p, a) = (precond_matrix5(&w, ur), flux_jacobian3(&w, n));
            for t in [0.01, 0.3, 2.0, -0.2] {
                let mut sum = p;
                for k in 0..25 {
                    sum[k] += t * a[k];
                }
                let dense = inv5(&sum);
                let mut diag = [0.0; 25];
                for i in 0..5 {
                    for j in 0..5 {
                        diag[5 * i + j] = (0..5).map(|q| e.r[i][q] / (1.0 + t * e.lam[q]) * e.l_pinv[q][j]).sum();
                    }
                }
                assert!(max_abs(&dense, &diag) < 1e-9 * (1.0 + dense.iter().fold(0.0f64, |m, x| m.max(x.abs()))), "t = {t}: {}", max_abs(&dense, &diag));
            }
        }
    }

    /// The face dissipation `P |P^-1 A|` of a state against itself is `P R diag(e) L` with the entropy-fixed `|lambda'|` of `roe_eigs_pc5` (the polynomial interpolation
    /// property), and `face_dis_pc5` returns those eigenvalues without a shock; rotating state and normal rotates it.
    #[test]
    fn dissipation_is_p_times_the_absolute_preconditioned_jacobian() {
        let mut rng = Rng::new(6);
        let lm = LowMach { ur_min: 0.1 };
        let p = ImplicitParams { low_mach_precond: true, ..ImplicitParams::DEFAULT };
        for _ in 0..100 {
            let w = state(&mut rng, 0.4);
            let n = unit(&mut rng);
            let c = sound_speed3(&w);
            let ur = lm.u_ref(&w, c);
            let d = abs_jacobian_pc3(&w, &w, n, 0.1, &lm);
            let (e1, e2, e3) = roe_eigs_pc5(&w, &w, n, 0.1, &lm);
            let eg = eig_pc5(&w, n, ur);
            let (r, l, pm) = (to_m5(&eg.r), to_m5(&eg.l), precond_matrix5(&w, ur));
            let mut rd = r;
            for i in 0..5 {
                for j in 0..5 {
                    rd[5 * i + j] *= [e1, e1, e1, e2, e3][j];
                }
            }
            let want = mm(&pm, &mm(&rd, &l));
            assert!(max_abs(&d, &want) < 1e-10 * (1.0 + want.iter().fold(0.0f64, |m, x| m.max(x.abs()))), "{}", max_abs(&d, &want));
            let fd = face_dis_pc5(&w, &w, n, &p, &lm);
            assert_eq!(fd, [e1, e1, e1, e2, e3]);
            // rotation: Q A Q^T for the conservative variables rotates the dissipation
            let q = random_rotation(&mut rng);
            let (wr, nr) = ([w[0], rot(&q, [w[1], w[2], w[3]])[0], rot(&q, [w[1], w[2], w[3]])[1], rot(&q, [w[1], w[2], w[3]])[2], w[4]], rot(&q, n));
            let dr = abs_jacobian_pc3(&wr, &wr, nr, 0.1, &lm);
            let mut qm = EYE;
            for a in 0..3 {
                for b in 0..3 {
                    qm[5 * (1 + a) + 1 + b] = q[a][b];
                }
            }
            let rot_d = mm(
                &qm,
                &mm(&d, &{
                    let mut qt = EYE;
                    for a in 0..3 {
                        for b in 0..3 {
                            qt[5 * (1 + a) + 1 + b] = q[b][a];
                        }
                    }
                    qt
                }),
            );
            assert!(max_abs(&dr, &rot_d) < 1e-10 * (1.0 + dr.iter().fold(0.0f64, |m, x| m.max(x.abs()))));
        }
    }

    /// The preconditioned pseudo-time step radius at low Mach is far below the acoustic one: `|u'| + c'` of a Mach 0.01 state with `U_r = 0.1 c_inf` is `~ 0.15 c`
    /// against `|u_n| + c`: the pseudo time step grows by the ratio, which is what the preconditioning is for.
    #[test]
    fn preconditioned_spectral_radius_at_low_mach() {
        let fs: Prim3 = [1.0, 0.01, 0.0, 0.0, 1.0 / GAMMA];
        let lm = LowMach::for_freestream(&fs);
        assert!((lm.ur_min - 0.1).abs() < 1e-12);
        let r = spectral_pc3(&fs, [1.0, 0.0, 0.0], &lm);
        assert!(r < 0.12 && r > 0.1, "{r}");
        assert!(LowMach::of(&ImplicitParams::DEFAULT, &fs).is_none());
        assert!(LowMach::of(&ImplicitParams { low_mach_precond: true, ..ImplicitParams::DEFAULT }, &fs).is_some());
        // supersonic free stream: P = I everywhere near it
        let sup: Prim3 = [1.0, 2.0, 0.0, 0.0, 1.0 / GAMMA];
        let lms = LowMach::for_freestream(&sup);
        assert!(max_abs(&lms.p_matrix(&sup), &EYE) == 0.0);
    }

    /// The point of the exercise: at Mach 0.01 with `U_r = 0.1 c` the spread `max |lambda| / min |lambda|` of the eigenvalues of the operator falls from `~ 100` (`u_n`,
    /// `u_n +- c`) to `~ 10` (`u_n`, `u' +- c'`), the factor by which the pseudo time step of the slowest wave can grow.
    #[test]
    fn preconditioning_reduces_the_eigenvalue_spread() {
        let w: Prim3 = [1.0, 0.01, 0.0, 0.0, 1.0 / GAMMA];
        let n = [1.0, 0.0, 0.0];
        let lm = LowMach { ur_min: 0.1 };
        let spread = |lam: &[f64; 5]| lam.iter().fold(0.0f64, |m, l| m.max(l.abs())) / lam.iter().fold(f64::INFINITY, |m, l| m.min(l.abs()));
        let c = sound_speed3(&w);
        let plain = spread(&[0.01, 0.01, 0.01, 0.01 + c, 0.01 - c]);
        let pc = spread(&lam_pc5(&w, n, &lm));
        assert!(plain > 90.0 && pc < 12.0 && pc > 9.0, "spread {plain} -> {pc}");
    }
}
