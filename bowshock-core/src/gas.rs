//! Ideal-gas helpers. State layouts:
//! * conservative `Cons = [rho, rho*u, rho*v, E]`, `E = p/(g-1) + rho (u^2+v^2)/2`
//! * primitive    `Prim = [rho, u, v, p]`
//! with `u` the axial (x) and `v` the radial (r) velocity.

pub const GAMMA: f64 = 1.4;

pub type Cons = [f64; 4];
pub type Prim = [f64; 4];

#[inline]
pub fn prim_to_cons(w: &Prim) -> Cons {
    let [rho, u, v, p] = *w;
    [rho, rho * u, rho * v, p / (GAMMA - 1.0) + 0.5 * rho * (u * u + v * v)]
}

#[inline]
pub fn cons_to_prim(q: &Cons) -> Prim {
    let rho = q[0];
    let u = q[1] / rho;
    let v = q[2] / rho;
    let p = (GAMMA - 1.0) * (q[3] - 0.5 * rho * (u * u + v * v));
    [rho, u, v, p]
}

#[inline]
pub fn sound_speed(w: &Prim) -> f64 {
    (GAMMA * w[3] / w[0]).sqrt()
}

/// Physical Euler flux projected on the unit normal `(nx, nr)`: `F.n`.
#[inline]
pub fn physical_flux(w: &Prim, nx: f64, nr: f64) -> Cons {
    let [rho, u, v, p] = *w;
    let un = u * nx + v * nr;
    let e = p / (GAMMA - 1.0) + 0.5 * rho * (u * u + v * v);
    [rho * un, rho * u * un + p * nx, rho * v * un + p * nr, (e + p) * un]
}

/// Free stream in non-dimensional units: rho=1, c=1 (so p=1/gamma), u=Mach, v=0.
pub fn freestream(mach: f64) -> Prim {
    [1.0, mach, 0.0, 1.0 / GAMMA]
}

/// Reflect the velocity about a wall with unit normal `(nx, nr)`.
#[inline]
pub fn reflect(w: &Prim, nx: f64, nr: f64) -> Prim {
    let un = w[1] * nx + w[2] * nr;
    [w[0], w[1] - 2.0 * un * nx, w[2] - 2.0 * un * nr, w[3]]
}
