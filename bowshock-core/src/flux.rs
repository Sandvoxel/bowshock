//! Face kernels: slope limiters, MUSCL reconstruction and the HLLC Riemann flux.

use crate::gas::{physical_flux, Cons, Prim, GAMMA};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Limiter {
    /// Piecewise-constant (first order).
    FirstOrder,
    Minmod,
    VanAlbada,
}

/// Limited cell slope `phi(a, b)` with `a = q_i - q_{i-1}`, `b = q_{i+1} - q_i`.
/// Face values are `q_i +- phi/2`.
#[inline]
pub fn limited_slope(a: f64, b: f64, lim: Limiter) -> f64 {
    match lim {
        Limiter::FirstOrder => 0.0,
        Limiter::Minmod => {
            if a * b <= 0.0 {
                0.0
            } else if a.abs() < b.abs() {
                a
            } else {
                b
            }
        }
        Limiter::VanAlbada => {
            // smooth van Albada: (a(b^2+e) + b(a^2+e)) / (a^2+b^2+2e); == a when a == b
            const EPS: f64 = 1e-14;
            if a * b <= 0.0 {
                0.0
            } else {
                (a * (b * b + EPS) + b * (a * a + EPS)) / (a * a + b * b + 2.0 * EPS)
            }
        }
    }
}

/// MUSCL reconstruction of the primitive variables at the face between cells
/// `m1` (left) and `p0` (right); `m2` and `p1` are the outer neighbours.
/// Falls back to first order for a face whose reconstructed density or pressure is not positive.
#[inline]
pub fn reconstruct(m2: &Prim, m1: &Prim, p0: &Prim, p1: &Prim, lim: Limiter) -> (Prim, Prim) {
    let mut l = *m1;
    let mut r = *p0;
    for k in 0..4 {
        let phi_l = limited_slope(m1[k] - m2[k], p0[k] - m1[k], lim);
        let phi_r = limited_slope(p0[k] - m1[k], p1[k] - p0[k], lim);
        l[k] = m1[k] + 0.5 * phi_l;
        r[k] = p0[k] - 0.5 * phi_r;
    }
    if l[0] <= 0.0 || l[3] <= 0.0 {
        l = *m1;
    }
    if r[0] <= 0.0 || r[3] <= 0.0 {
        r = *p0;
    }
    (l, r)
}

/// HLLC flux (Toro, *Riemann Solvers and Numerical Methods for Fluid Dynamics*, ch. 10)
/// for the 2D Euler equations, projected on the unit normal `(nx, nr)`.
/// Wave speeds: Einfeldt/Roe-averaged estimates `SL = min(unL-cL, u~-c~)`, `SR = max(unR+cR, u~+c~)`.
/// Returns the flux per unit face length `F.n` (no geometric weighting).
#[inline]
pub fn hllc(wl: &Prim, wr: &Prim, nx: f64, nr: f64) -> Cons {
    let [rl, ul, vl, pl] = *wl;
    let [rr, ur, vr, pr] = *wr;
    let unl = ul * nx + vl * nr;
    let unr = ur * nx + vr * nr;
    let cl = (GAMMA * pl / rl).sqrt();
    let cr = (GAMMA * pr / rr).sqrt();
    let el = pl / (GAMMA - 1.0) + 0.5 * rl * (ul * ul + vl * vl);
    let er = pr / (GAMMA - 1.0) + 0.5 * rr * (ur * ur + vr * vr);

    // Roe averages
    let sql = rl.sqrt();
    let sqr = rr.sqrt();
    let inv = 1.0 / (sql + sqr);
    let uh = (sql * ul + sqr * ur) * inv;
    let vh = (sql * vl + sqr * vr) * inv;
    let hl = (el + pl) / rl;
    let hr = (er + pr) / rr;
    let hh = (sql * hl + sqr * hr) * inv;
    let ch2 = (GAMMA - 1.0) * (hh - 0.5 * (uh * uh + vh * vh));
    let ch = ch2.max(1e-300).sqrt();
    let unh = uh * nx + vh * nr;

    let sl = (unl - cl).min(unh - ch);
    let sr = (unr + cr).max(unh + ch);
    if sl >= 0.0 {
        return physical_flux(wl, nx, nr);
    }
    if sr <= 0.0 {
        return physical_flux(wr, nx, nr);
    }

    let sm = (pr - pl + rl * unl * (sl - unl) - rr * unr * (sr - unr)) / (rl * (sl - unl) - rr * (sr - unr));

    if sm >= 0.0 {
        let f = physical_flux(wl, nx, nr);
        let fac = rl * (sl - unl) / (sl - sm);
        let ul_star = [fac, fac * (ul + (sm - unl) * nx), fac * (vl + (sm - unl) * nr), fac * (el / rl + (sm - unl) * (sm + pl / (rl * (sl - unl))))];
        let q = [rl, rl * ul, rl * vl, el];
        [f[0] + sl * (ul_star[0] - q[0]), f[1] + sl * (ul_star[1] - q[1]), f[2] + sl * (ul_star[2] - q[2]), f[3] + sl * (ul_star[3] - q[3])]
    } else {
        let f = physical_flux(wr, nx, nr);
        let fac = rr * (sr - unr) / (sr - sm);
        let ur_star = [fac, fac * (ur + (sm - unr) * nx), fac * (vr + (sm - unr) * nr), fac * (er / rr + (sm - unr) * (sm + pr / (rr * (sr - unr))))];
        let q = [rr, rr * ur, rr * vr, er];
        [f[0] + sr * (ur_star[0] - q[0]), f[1] + sr * (ur_star[1] - q[1]), f[2] + sr * (ur_star[2] - q[2]), f[3] + sr * (ur_star[3] - q[3])]
    }
}

/// Rusanov (local Lax-Friedrichs) flux, projected on `(nx, nr)`: `0.5 (F_L + F_R) - 0.5 smax (U_R - U_L)`
/// with `smax = max(|un_L| + c_L, |un_R| + c_R)`. Dissipates every wave family (incl. contact/shear) at the
/// maximum rate.
#[inline]
pub fn rusanov(wl: &Prim, wr: &Prim, nx: f64, nr: f64) -> Cons {
    let [rl, ul, vl, pl] = *wl;
    let [rr, ur, vr, pr] = *wr;
    let unl = ul * nx + vl * nr;
    let unr = ur * nx + vr * nr;
    let smax = (unl.abs() + (GAMMA * pl / rl).sqrt()).max(unr.abs() + (GAMMA * pr / rr).sqrt());
    let fl = physical_flux(wl, nx, nr);
    let fr = physical_flux(wr, nx, nr);
    let ql = [rl, rl * ul, rl * vl, pl / (GAMMA - 1.0) + 0.5 * rl * (ul * ul + vl * vl)];
    let qr = [rr, rr * ur, rr * vr, pr / (GAMMA - 1.0) + 0.5 * rr * (ur * ur + vr * vr)];
    let mut out = [0.0; 4];
    for k in 0..4 {
        out[k] = 0.5 * (fl[k] + fr[k]) - 0.5 * smax * (qr[k] - ql[k]);
    }
    out
}

/// Shock-sensor lower/upper threshold on the relative cell-to-cell pressure jump
/// `|p_R - p_L| / min(p_R, p_L)` (see [`shock_weight`]).
pub const SHOCK_SENSOR_LO: f64 = 0.002;
pub const SHOCK_SENSOR_HI: f64 = 0.02;

/// Weight `theta in [0, 1]` of the Rusanov flux in [`face_flux`]: smoothstep of the relative pressure jump
/// between the two cells adjacent to the face (Quirk-type shock sensor). `0` in smooth flow (pure HLLC),
/// `1` inside a captured shock (pure Rusanov); C1-continuous so the steady state stays a smooth fixed point.
#[inline]
pub fn shock_weight(pl: f64, pr: f64) -> f64 {
    let s = (pr - pl).abs() / pl.min(pr);
    let t = ((s - SHOCK_SENSOR_LO) / (SHOCK_SENSOR_HI - SHOCK_SENSOR_LO)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Complete face kernel: MUSCL reconstruction + HLLC, blended towards Rusanov inside shocks.
/// `m2, m1 | p0, p1` are the four primitive cell states around the face (`m1`/`p0` adjacent to it).
///
/// Why the blend: HLLC resolves contact/shear waves with zero dissipation; combined with second-order
/// reconstruction a captured oblique shock that is not aligned with the grid is a (convectively) unstable
/// discrete structure: perturbations in the shock cells grow with distance downstream and saturate as a
/// limit cycle (README, "Convergence stall"). Rusanov at shock faces removes it; away from shocks the
/// accurate HLLC flux is used unchanged.
#[inline]
pub fn face_flux(m2: &Prim, m1: &Prim, p0: &Prim, p1: &Prim, lim: Limiter, nx: f64, nr: f64) -> Cons {
    let (l, r) = reconstruct(m2, m1, p0, p1, lim);
    let c = hllc(&l, &r, nx, nr);
    let t = shock_weight(m1[3], p0[3]);
    if t == 0.0 {
        return c;
    }
    let f = rusanov(&l, &r, nx, nr);
    let mut out = c;
    for k in 0..4 {
        out[k] = (1.0 - t) * c[k] + t * f[k];
    }
    out
}

/// Lower bound of the low-Mach scaling factor `z` in [`low_mach_states`].
pub const LOW_MACH_FLOOR: f64 = 0.05;

/// Low-Mach correction of the reconstructed face states (Thornber et al., "An improved reconstruction method for
/// compressible flows with low Mach number features", J. Comput. Phys. 227 (2008) 4873): the jump of the face-normal
/// velocity between the left and right state is scaled by `z = min(max(M_L, M_R), 1)` (`M = |V|/c` of the reconstructed
/// states, floored at [`LOW_MACH_FLOOR`]):
///
/// ```text
/// u_n,L' = u_n,avg + z (u_n,L - u_n,avg),     u_n,R' = u_n,avg + z (u_n,R - u_n,avg)
/// ```
///
/// Without it the dissipation of an upwind Riemann solver contains a pressure term `~ rho c Delta u_n` that is `O(1/M)`
/// larger than the dynamic pressure, so the pressure field degrades as `M -> 0` (and its error does not vanish with
/// the mesh at fixed `M`). Only the normal velocity is touched (tangential velocity, density and pressure are unchanged),
/// so a uniform state, a stationary contact and any state with `u_n,L = u_n,R` are unchanged; `z = 1` for `M >= 1`, i.e.
/// the correction is inactive in supersonic regions. `p`, `rho` and the tangential velocity are not modified.
#[inline]
pub fn low_mach_states(l: &Prim, r: &Prim, nx: f64, nr: f64) -> (Prim, Prim) {
    let unl = l[1] * nx + l[2] * nr;
    let unr = r[1] * nx + r[2] * nr;
    let ml = (l[1] * l[1] + l[2] * l[2]).sqrt() / (GAMMA * l[3] / l[0]).sqrt();
    let mr = (r[1] * r[1] + r[2] * r[2]).sqrt() / (GAMMA * r[3] / r[0]).sqrt();
    let z = ml.max(mr).clamp(LOW_MACH_FLOOR, 1.0);
    if z >= 1.0 {
        return (*l, *r);
    }
    let avg = 0.5 * (unl + unr);
    let dl = avg + z * (unl - avg) - unl;
    let dr = avg + z * (unr - avg) - unr;
    ([l[0], l[1] + dl * nx, l[2] + dl * nr, l[3]], [r[0], r[1] + dr * nx, r[2] + dr * nr, r[3]])
}

/// [`face_flux`] with the optional low-Mach correction [`low_mach_states`] applied to the reconstructed states before the
/// Riemann solvers; `low_mach = false` is exactly [`face_flux`].
#[inline]
pub fn face_flux_lm(m2: &Prim, m1: &Prim, p0: &Prim, p1: &Prim, lim: Limiter, nx: f64, nr: f64, low_mach: bool) -> Cons {
    if !low_mach {
        return face_flux(m2, m1, p0, p1, lim, nx, nr);
    }
    let (l, r) = reconstruct(m2, m1, p0, p1, lim);
    let (l, r) = low_mach_states(&l, &r, nx, nr);
    let c = hllc(&l, &r, nx, nr);
    let t = shock_weight(m1[3], p0[3]);
    if t == 0.0 {
        return c;
    }
    let f = rusanov(&l, &r, nx, nr);
    let mut out = c;
    for k in 0..4 {
        out[k] = (1.0 - t) * c[k] + t * f[k];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hllc_consistency() {
        // F(U,U) must equal the physical flux for any state and normal direction.
        let states: [Prim; 5] = [[1.0, 2.0, 0.0, 1.0 / GAMMA], [0.3, -1.5, 0.7, 0.2], [2.5, 0.0, 0.0, 3.0], [1.2, 0.3, -2.4, 0.9], [0.9, 5.0, 3.0, 1.3]];
        for w in &states {
            for k in 0..16 {
                let a = 0.4 * k as f64;
                let (nr, nx) = a.sin_cos();
                let f = hllc(w, w, nx, nr);
                let g = physical_flux(w, nx, nr);
                for c in 0..4 {
                    assert!((f[c] - g[c]).abs() <= 1e-13 * (1.0 + g[c].abs()), "{w:?} {a} {c}");
                }
            }
        }
    }

    #[test]
    fn hllc_resolves_stationary_contact() {
        // Stationary contact (equal p, u=0) must carry zero mass/energy flux, pressure-only momentum flux.
        let l: Prim = [1.0, 0.0, 0.0, 1.0];
        let r: Prim = [0.125, 0.0, 0.0, 1.0];
        let f = hllc(&l, &r, 1.0, 0.0);
        assert!(f[0].abs() < 1e-14 && f[3].abs() < 1e-14);
        assert!((f[1] - 1.0).abs() < 1e-14 && f[2].abs() < 1e-14);
    }

    #[test]
    fn limiters_are_consistent() {
        for lim in [Limiter::Minmod, Limiter::VanAlbada] {
            assert!((limited_slope(0.7, 0.7, lim) - 0.7).abs() < 1e-12);
            assert_eq!(limited_slope(1.0, -1.0, lim), 0.0);
        }
        assert_eq!(limited_slope(1.0, 3.0, Limiter::Minmod), 1.0);
    }

    #[test]
    fn rusanov_consistency_and_shock_blend() {
        let w: Prim = [0.9, 5.0, 3.0, 1.3];
        for k in 0..8 {
            let (nr, nx) = (0.8 * k as f64).sin_cos();
            let (f, g) = (rusanov(&w, &w, nx, nr), physical_flux(&w, nx, nr));
            for c in 0..4 {
                assert!((f[c] - g[c]).abs() <= 1e-13 * (1.0 + g[c].abs()));
            }
        }
        // smooth flow: pure HLLC; strong jump: pure Rusanov; monotone in between
        assert_eq!(shock_weight(1.0, 1.0 + 0.5 * SHOCK_SENSOR_LO), 0.0);
        assert_eq!(shock_weight(1.0, 1.5), 1.0);
        assert!(shock_weight(1.0, 1.0 + 0.5 * (SHOCK_SENSOR_LO + SHOCK_SENSOR_HI)) > 0.4);
        let (a, b): (Prim, Prim) = ([1.0, 2.0, 0.0, 0.7], [1.5, 1.7, 0.2, 1.0]);
        let f = face_flux(&a, &a, &b, &b, Limiter::FirstOrder, 1.0, 0.0);
        let r = rusanov(&a, &b, 1.0, 0.0);
        for c in 0..4 {
            assert!((f[c] - r[c]).abs() < 1e-14);
        }
    }

    #[test]
    fn low_mach_flux_is_consistent_and_reduces_to_the_standard_flux() {
        // F(U, U) = F(U) with the correction on (the normal-velocity jump is zero)
        let w: Prim = [1.1, 0.04, -0.02, 0.8];
        for k in 0..8 {
            let (nr, nx) = (0.8 * k as f64).sin_cos();
            let f = face_flux_lm(&w, &w, &w, &w, Limiter::VanAlbada, nx, nr, true);
            let g = physical_flux(&w, nx, nr);
            for c in 0..4 {
                assert!((f[c] - g[c]).abs() <= 1e-13 * (1.0 + g[c].abs()), "{k} {c}");
            }
        }
        // low_mach = false is exactly the standard kernel
        let (a, b, c, d): (Prim, Prim, Prim, Prim) = ([1.0, 0.1, 0.0, 0.7], [1.01, 0.12, 0.01, 0.71], [1.02, 0.15, 0.02, 0.72], [1.04, 0.2, 0.0, 0.73]);
        assert_eq!(face_flux_lm(&a, &b, &c, &d, Limiter::Minmod, 0.6, 0.8, false), face_flux(&a, &b, &c, &d, Limiter::Minmod, 0.6, 0.8));
        // supersonic faces (M >= 1 on either side): z = 1, the states are unchanged -> bit-identical flux
        let (l, r): (Prim, Prim) = ([1.0, 1.3, 0.0, 0.714], [1.1, 1.5, 0.1, 0.8]);
        assert_eq!(low_mach_states(&l, &r, 1.0, 0.0), (l, r));
        assert_eq!(face_flux_lm(&l, &l, &r, &r, Limiter::FirstOrder, 1.0, 0.0, true), face_flux(&l, &l, &r, &r, Limiter::FirstOrder, 1.0, 0.0));
    }

    #[test]
    fn low_mach_scaling_reduces_the_normal_velocity_jump_only() {
        let (l, r): (Prim, Prim) = ([1.0, 0.12, 0.05, 0.714], [1.0, 0.08, 0.05, 0.714]);
        let (lm, rm) = low_mach_states(&l, &r, 1.0, 0.0);
        let m_local = (0.12f64.hypot(0.05)) / (GAMMA * 0.714f64).sqrt();
        // jump scaled by z = M_local, mean preserved, everything but u unchanged
        assert!(((rm[1] - lm[1]) - m_local * (0.08 - 0.12)).abs() < 1e-14);
        assert!((0.5 * (lm[1] + rm[1]) - 0.10).abs() < 1e-14);
        assert_eq!((lm[0], lm[2], lm[3]), (l[0], l[2], l[3]));
        assert_eq!((rm[0], rm[2], rm[3]), (r[0], r[2], r[3]));
        // z is floored: a near-stagnant pair still keeps LOW_MACH_FLOOR of the jump
        let (a, b): (Prim, Prim) = ([1.0, 1e-6, 0.0, 0.714], [1.0, -1e-6, 0.0, 0.714]);
        let (am, bm) = low_mach_states(&a, &b, 1.0, 0.0);
        assert!(((bm[1] - am[1]) - LOW_MACH_FLOOR * (-2e-6)).abs() < 1e-18);
    }
}
