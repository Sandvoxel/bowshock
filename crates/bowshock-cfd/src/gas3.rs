//! 3D ideal-gas helpers, the Cartesian counterpart of [`crate::gas`]. State layouts:
//! * conservative `Cons3 = [rho, rho*u, rho*v, rho*w, E]`, `E = p/(g-1) + rho (u^2+v^2+w^2)/2`
//! * primitive    `Prim3 = [rho, u, v, w, p]`
//!
//! Velocities are Cartesian: `u` axial (x), `v` lateral (y), `w` vertical (z). Incidence `alpha > 0` means the air moves
//! towards +z relative to the body, so the windward side is -z and a positive normal force points to +z.

pub use crate::gas::GAMMA;

pub type Cons3 = [f64; 5];
pub type Prim3 = [f64; 5];

#[inline]
pub fn prim_to_cons3(w: &Prim3) -> Cons3 {
    let [rho, u, v, ww, p] = *w;
    [rho, rho * u, rho * v, rho * ww, p / (GAMMA - 1.0) + 0.5 * rho * (u * u + v * v + ww * ww)]
}

#[inline]
pub fn cons_to_prim3(q: &Cons3) -> Prim3 {
    let rho = q[0];
    let (u, v, w) = (q[1] / rho, q[2] / rho, q[3] / rho);
    let p = (GAMMA - 1.0) * (q[4] - 0.5 * rho * (u * u + v * v + w * w));
    [rho, u, v, w, p]
}

#[inline]
pub fn sound_speed3(w: &Prim3) -> f64 {
    (GAMMA * w[4] / w[0]).sqrt()
}

/// Physical Euler flux projected on the unit normal `n`: `F.n`.
#[inline]
pub fn physical_flux3(w: &Prim3, n: [f64; 3]) -> Cons3 {
    let [rho, u, v, ww, p] = *w;
    let un = u * n[0] + v * n[1] + ww * n[2];
    let e = p / (GAMMA - 1.0) + 0.5 * rho * (u * u + v * v + ww * ww);
    [rho * un, rho * u * un + p * n[0], rho * v * un + p * n[1], rho * ww * un + p * n[2], (e + p) * un]
}

/// Free stream in non-dimensional units (rho=1, c=1, p=1/gamma) at incidence `alpha` \[rad\]: velocity `M (cos a, 0, sin a)`.
pub fn freestream3(mach: f64, alpha: f64) -> Prim3 {
    [1.0, mach * alpha.cos(), 0.0, mach * alpha.sin(), 1.0 / GAMMA]
}

/// Reflect the velocity about a wall with unit normal `n`.
#[inline]
pub fn reflect3(w: &Prim3, n: [f64; 3]) -> Prim3 {
    let un = w[1] * n[0] + w[2] * n[1] + w[3] * n[2];
    [w[0], w[1] - 2.0 * un * n[0], w[2] - 2.0 * un * n[1], w[3] - 2.0 * un * n[2], w[4]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas;

    #[test]
    fn round_trip() {
        let w = [1.3, 0.7, -0.2, 0.4, 0.9];
        let back = cons_to_prim3(&prim_to_cons3(&w));
        for k in 0..5 {
            assert!((w[k] - back[k]).abs() < 1e-14);
        }
    }

    #[test]
    fn flux_matches_2d_in_xz_plane() {
        let (nx, nr) = (0.6, 0.8);
        let w3 = [1.2, 0.9, 0.0, -0.35, 0.7];
        let f3 = physical_flux3(&w3, [nx, 0.0, nr]);
        let f2 = gas::physical_flux(&[w3[0], w3[1], w3[3], w3[4]], nx, nr);
        assert_eq!(f3[2], 0.0);
        for (a, b) in [(f3[0], f2[0]), (f3[1], f2[1]), (f3[3], f2[2]), (f3[4], f2[3])] {
            assert!((a - b).abs() < 1e-14);
        }
        let r3 = reflect3(&w3, [nx, 0.0, nr]);
        let r2 = gas::reflect(&[w3[0], w3[1], w3[3], w3[4]], nx, nr);
        assert!((r3[1] - r2[1]).abs() < 1e-14 && (r3[3] - r2[2]).abs() < 1e-14);
        let fs = freestream3(2.0, 0.1);
        assert!((fs[1] - 2.0 * 0.1f64.cos()).abs() < 1e-15 && (fs[3] - 2.0 * 0.1f64.sin()).abs() < 1e-15);
    }
}
