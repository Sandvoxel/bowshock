// The line-implicit scheme `Scheme::Implicit` in array ops. This file is `include!`d by `gpu/mod.rs` after `body.rs` into the same
// per-precision module, so it sees `S`, `A`, `Rec`, `State`, `LazySolver`, `lhs` / `from_lhs` (the f32 view of an `A` and back)
// and everything `body.rs` imports.
//
// It mirrors `crate::implicit` (`flux_jacobian`, `abs_jacobian`, `cell_blocks`, `assemble`, `solve_lines`,
// `apply_factor`, `step`, `compute_dt`), inviscid case, grids without a solid block. One iteration solves the backward-Euler
// pseudo-time step with the first-order flux Jacobians on the left,
//
//     (I - dt J_i) X = dt R,   (I - dt J_j) dU = X,   U <- U + alpha dU,
//
// `R` being the residual of the full second-order scheme (`LazySolver::stage_from`, in the precision `S`). The left-hand side
// (Jacobians, dissipation, ghost Jacobians, dt, line solves, defect correction) is plain f32 like the hand-written kernels'
// (`bowshock-gpu/src/shaders_impl.rs`): it only steers the iteration, the fixed point is that of the `S`-precision `R`.
//
// A 4x4 block is `Blk`: sixteen entries, row-major like the CPU `M4`, each an array over the cells or faces (`E::A`) or the
// constants 0 / 1 (`E::Z`, `E::O`), so that the structurally zero entries of the flux Jacobian cost nothing. The line solves are
// Luma's `solve_block_tridiagonal_parts` on the sixteen entries of each block (no stacking); a vector field is four `[ni, nj]` arrays.

use crate::solver::Scheme;

/// Four f32 fields (primitive state, or a conserved vector).
type V4 = [F; 4];

/// A 4x4 block, row-major (`m[4 r + c]`, the CPU `M4`), one entry per row / column pair.
#[derive(Clone)]
struct Blk([E; 16]);

impl Blk {
    fn zero() -> Blk {
        Blk(std::array::from_fn(|_| E::Z))
    }

    fn eye() -> Blk {
        Blk(std::array::from_fn(|k| if k % 5 == 0 { E::O } else { E::Z }))
    }

    fn zip(&self, o: &Blk, f: impl Fn(&E, &E) -> E) -> Blk {
        Blk(std::array::from_fn(|k| f(&self.0[k], &o.0[k])))
    }

    fn add(&self, o: &Blk) -> Blk {
        self.zip(o, E::add)
    }

    fn sub(&self, o: &Blk) -> Blk {
        self.zip(o, E::sub)
    }

    /// Every entry times the array `s`.
    fn scale(&self, s: &F) -> Blk {
        Blk(std::array::from_fn(|k| self.0[k].scale(s)))
    }

    /// `self + d I`.
    fn add_diag(&self, d: &F) -> Blk {
        Blk(std::array::from_fn(|k| if k % 5 == 0 { self.0[k].add(&E::A(d.clone())) } else { self.0[k].clone() }))
    }

    /// `self - d I`.
    fn sub_diag(&self, d: &F) -> Blk {
        Blk(std::array::from_fn(|k| if k % 5 == 0 { self.0[k].sub(&E::A(d.clone())) } else { self.0[k].clone() }))
    }

    /// `self o`: `c[4 i + j] = sum_k a[4 i + k] b[4 k + j]`, `k` ascending like the CPU `mm`.
    fn mm(&self, o: &Blk) -> Blk {
        Blk(std::array::from_fn(|n| {
            let (i, j) = (n / 4, n % 4);
            (0..4).fold(E::Z, |acc, k| acc.add(&self.0[4 * i + k].mul(&o.0[4 * k + j])))
        }))
    }

    /// The part `range` of every array entry along `axis`.
    fn slice(&self, axis: usize, range: std::ops::Range<usize>) -> Blk {
        Blk(std::array::from_fn(|k| match &self.0[k] {
            E::A(a) => E::A(a.slice(axis, range.clone())),
            e => e.clone(),
        }))
    }

    /// All sixteen entries as arrays (`z` and `o` the arrays of zeros and ones of the shape).
    fn dense(&self, z: &F, o: &F) -> [F; 16] {
        std::array::from_fn(|k| match &self.0[k] {
            E::Z => z.clone(),
            E::O => o.clone(),
            E::A(a) => a.clone(),
        })
    }

    fn of(dense: &[F; 16]) -> Blk {
        Blk(std::array::from_fn(|k| E::A(dense[k].clone())))
    }
}

/// `y = a x` for a block of arrays: `y_i = a_i0 x_0 + a_i1 x_1 + a_i2 x_2 + a_i3 x_3`.
fn mv(a: &[F; 16], x: &V4) -> V4 {
    std::array::from_fn(|i| &a[4 * i] * &x[0] + &a[4 * i + 1] * &x[1] + &a[4 * i + 2] * &x[2] + &a[4 * i + 3] * &x[3])
}


// ---------------------------------------------------------------------------------------------------------------------
// low-Mach preconditioning (`ImplicitParams::low_mach_precond`; the CPU counterpart is `crate::implicit::LowMach`)
// ---------------------------------------------------------------------------------------------------------------------

/// The preconditioning of a run, f32 view of the CPU `LowMach`: Weiss-Smith `P` in both factors, the defect operator and between the sweeps,
/// `P |P^-1 A|` as the dissipation matrix, the preconditioned spectral radius in the local time step. Holds the lower bound of the reference
/// velocity `U_r` squared.
#[derive(Clone, Copy)]
struct LmGpu {
    ur_min2: f32,
}

impl LmGpu {
    /// The preconditioning of the free stream `fs` when `p.low_mach_precond` is set.
    fn of(p: &ImplicitParams, fs: &[f64; 4]) -> Option<LmGpu> {
        p.low_mach_precond.then(|| {
            let ur_min = crate::implicit::LowMach::for_freestream(fs).ur_min;
            LmGpu { ur_min2: (ur_min * ur_min) as f32 }
        })
    }

    /// `U_r^2 = clamp(|u|^2, U_min^2, c^2)` (`U_r = c`, i.e. `P = I`, is reached exactly).
    fn ur2(&self, q2: &F, c2: &F) -> F {
        q2.maximum(self.ur_min2).minimum(c2)
    }
}

/// `(g_0, g_1, g_2)` of `g = (g - 1)(q^2 / 2, -u, -v, 1)`: `g_3 = g - 1` is a constant.
fn pressure_row(u: &F, v: &F) -> [F; 3] {
    let g1 = cf(G1);
    [0.5f32 * g1 * (u * u + v * v), -(g1 * u), -(g1 * v)]
}

/// `(I + coef a g^T) X` for `a = (1, u, v, h)`, given `ca = coef a`: with `coef = kappa` the preconditioning matrix `P` applied to
/// `X`, with `coef = -kappa / (1 + kappa c^2)` its inverse (`g . a = c^2`). Rank one, so `g^T X` is one row of four dot products.
fn left_rank1(x: &Blk, ca: &[F; 4], g: &[F; 3]) -> Blk {
    let g1 = cf(G1);
    let row: [E; 4] = std::array::from_fn(|j| x.0[j].scale(&g[0]).add(&x.0[4 + j].scale(&g[1])).add(&x.0[8 + j].scale(&g[2])).add(&x.0[12 + j].muls(g1)));
    Blk(std::array::from_fn(|n| x.0[n].add(&row[n % 4].scale(&ca[n / 4]))))
}

/// `coef (1, u, v, h)`.
fn scaled_a(coef: &F, u: &F, v: &F, h: &F) -> [F; 4] {
    [coef.clone(), coef * u, coef * v, coef * h]
}

/// Dissipation matrix `P |P^-1 A|` of the Roe-averaged state (`abs_jacobian_pc`): `|M|` for `M = P^-1 A` (eigenvalues `a`
/// twice and `u' +- c'`) is the quadratic polynomial through the entropy-fixed `|a|`, `|u' + c'|`, `|u' - c'|` (Newton form), then
/// multiplied by `P`. `P` and `P^-1` are applied as rank-one updates (identical to the dense products). With `U_r = c` everywhere
/// in the argument (`b = 1`) this is the entry-for-entry formula of [`abs_jacobian_f`].
fn abs_jacobian_pc_f(wl: &V4, wr: &V4, nx: &F, nr: &F, eps_fix: f32, lm: &LmGpu) -> Blk {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0f32 / (&sl + &sr);
    let u = (&sl * &wl[1] + &sr * &wr[1]) * &inv;
    let v = (&sl * &wl[2] + &sr * &wr[2]) * &inv;
    let hl = cf(GAMMA) * &wl[3] / (cf(G1) * &wl[0]) + 0.5f32 * (&wl[1] * &wl[1] + &wl[2] * &wl[2]);
    let hr = cf(GAMMA) * &wr[3] / (cf(G1) * &wr[0]) + 0.5f32 * (&wr[1] * &wr[1] + &wr[2] * &wr[2]);
    let h = (&sl * &hl + &sr * &hr) * &inv;
    let q2 = &u * &u + &v * &v;
    let c2 = (cf(G1) * (&h - 0.5f32 * &q2)).maximum(1e-30f32);
    let c = c2.sqrt();
    let ur2 = lm.ur2(&q2, &c2);
    let a = &u * nx + &v * nr;
    // preconditioned eigenvalue data: b = U_r^2 / c^2, u' = a (1 + b) / 2, c' = sqrt(a^2 (1 - b)^2 + 4 b c^2) / 2
    let b = &ur2 / &c2;
    let u1 = 0.5f32 * &a * (1.0f32 + &b);
    let omb = 1.0f32 - &b;
    let c1 = 0.5f32 * (&a * &a * &omb * &omb + 4.0f32 * &b * &c2).sqrt();
    let (l2, l3) = (&u1 + &c1, &u1 - &c1);
    let eps = eps_fix * &c;
    let fix = |l: &F| -> F {
        let al = l.abs();
        select(&al.ge(&eps), &al, 0.5f32 * (l * l / &eps + &eps))
    };
    let (e1, e2, e3) = (fix(&a), fix(&l2), fix(&l3));
    let d1 = (&e2 - &e1) / (&l2 - &a);
    let d2 = ((&e3 - &e1) / (&l3 - &a) - &d1) / (&l3 - &l2);
    let kappa = kappa_f(&ur2, &c2);
    let f = &kappa / (1.0f32 + &kappa * &c2);
    let g = pressure_row(&u, &v);
    let am = flux_jacobian_h(&u, &v, &h, nx, nr);
    let m = left_rank1(&am, &scaled_a(&-&f, &u, &v, &h), &g);
    let b1 = m.sub_diag(&a);
    let b2 = m.sub_diag(&l2);
    let prod = b1.mm(&b2);
    let pm = b1.scale(&d1).add(&prod.scale(&d2)).add_diag(&e1);
    left_rank1(&pm, &scaled_a(&kappa, &u, &v, &h), &g)
}

/// The preconditioning matrix `P` of every cell (`[ni, nj]` entries, `P = I` where `U_r = c` and in solid cells) from the cell
/// primitives `wk`; `solid` masks the solid cells.
fn precond_p(wk: &V4, lm: &LmGpu, solid: Option<&Array<bool>>) -> [F; 16] {
    let [rho, u, v, p] = wk;
    let q2 = u * u + v * v;
    let c2 = cf(GAMMA) * p / rho;
    let h = cf(GAMMA) * p / (cf(G1) * rho) + 0.5f32 * &q2;
    let kappa = kappa_f(&lm.ur2(&q2, &c2), &c2);
    let ka = scaled_a(&kappa, u, v, &h);
    let g = pressure_row(u, v);
    let g1 = cf(G1);
    std::array::from_fn(|n| {
        let (i, j) = (n / 4, n % 4);
        let gj = if j < 3 { &ka[i] * &g[j] } else { &ka[i] * g1 };
        let e = if i == j { 1.0f32 + gj } else { gj };
        match solid {
            Some(s) => select(s, if i == j { 1.0f32 } else { 0.0f32 }, &e),
            None => e,
        }
    })
}

// ---------------------------------------------------------------------------------------------------------------------
// gas, flux Jacobian, dissipation
// ---------------------------------------------------------------------------------------------------------------------

/// `rho = q0, u = q1 / rho, v = q2 / rho, p = (g - 1) (E - rho (u^2 + v^2) / 2)` in f32.
fn cons_to_prim_f(q: &V4) -> V4 {
    let u = &q[1] / &q[0];
    let v = &q[2] / &q[0];
    let p = cf(G1) * (&q[3] - 0.5f32 * &q[0] * (&u * &u + &v * &v));
    [q[0].clone(), u, v, p]
}

/// `[rho, rho u, rho v, p / (g - 1) + rho (u^2 + v^2) / 2]` in f32.
fn prim_to_cons_f(w: &V4) -> V4 {
    let [rho, u, v, p] = w;
    [rho.clone(), rho * u, rho * v, p / cf(G1) + 0.5f32 * rho * (u * u + v * v)]
}

/// `A = d(F.n)/dU` of the primitive state `w` for the unit normal `(n_x, n_r)` (conservative variables `(rho, rho u, rho v, E)`):
/// `h = g p / ((g - 1) rho) + q^2 / 2` and the rows of [`flux_jacobian_h`].
fn flux_jacobian_f(w: &V4, nx: &F, nr: &F) -> Blk {
    let [rho, u, v, p] = w;
    let h = cf(GAMMA) * p / (cf(G1) * rho) + 0.5f32 * (u * u + v * v);
    flux_jacobian_h(u, v, &h, nx, nr)
}

/// [`flux_jacobian_f`] from the velocity and the total enthalpy `h` (the density does not enter `A(u, v, H)`):
///
/// ```text
/// th = u n_x + v n_r,  phi = (g - 1) q^2 / 2
/// [ 0                  n_x                       n_r                       0        ]
/// [ phi n_x - u th     th + (2 - g) u n_x        u n_r - (g - 1) v n_x     (g-1) n_x ]
/// [ phi n_r - v th     v n_x - (g - 1) u n_r     th + (2 - g) v n_r        (g-1) n_r ]
/// [ th (phi - h)       h n_x - (g - 1) u th      h n_r - (g - 1) v th      g th     ]
/// ```
fn flux_jacobian_h(u: &F, v: &F, h: &F, nx: &F, nr: &F) -> Blk {
    let g1 = cf(G1);
    let q2 = u * u + v * v;
    let th = u * nx + v * nr;
    let phi = (0.5f32 * g1) * &q2;
    let a = E::A;
    Blk([
        E::Z,
        a(nx.clone()),
        a(nr.clone()),
        E::Z,
        a(&phi * nx - u * &th),
        a(&th + (cf(2.0 - GAMMA) * u) * nx),
        a(u * nr - (g1 * v) * nx),
        a(g1 * nx),
        a(&phi * nr - v * &th),
        a(v * nx - (g1 * u) * nr),
        a(&th + (cf(2.0 - GAMMA) * v) * nr),
        a(g1 * nr),
        a(&th * (&phi - h)),
        a(h * nx - (g1 * u) * &th),
        a(h * nr - (g1 * v) * &th),
        a(cf(GAMMA) * &th),
    ])
}

/// `|u_n| + c` of the primitive state `w` for the unit normal `(n_x, n_r)`.
fn spectral_f(w: &V4, nx: &F, nr: &F) -> F {
    (&w[1] * nx + &w[2] * nr).abs() + (cf(GAMMA) * &w[3] / &w[0]).sqrt()
}

/// `|A|` of the Roe-averaged state of `wl`, `wr` for the unit normal `(n_x, n_r)`: `|A| = d1 (A - a I) + d2 (A - a I)(A - (a + c) I) + e1 I`
/// with `a = u~_n`, the entropy-fixed eigenvalue moduli `e1 = fix(a)`, `e2 = fix(a + c)`, `e3 = fix(a - c)` (`fix(l) = |l|` for
/// `|l| >= eps_fix c`, else `(l^2 / (eps_fix c) + eps_fix c) / 2`), `d1 = (e2 - e1) / c`, `d2 = (e2 + e3 - 2 e1) / (2 c^2)`.
fn abs_jacobian_f(wl: &V4, wr: &V4, nx: &F, nr: &F, eps_fix: f32) -> Blk {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0f32 / (&sl + &sr);
    let u = (&sl * &wl[1] + &sr * &wr[1]) * &inv;
    let v = (&sl * &wl[2] + &sr * &wr[2]) * &inv;
    let hl = cf(GAMMA) * &wl[3] / (cf(G1) * &wl[0]) + 0.5f32 * (&wl[1] * &wl[1] + &wl[2] * &wl[2]);
    let hr = cf(GAMMA) * &wr[3] / (cf(G1) * &wr[0]) + 0.5f32 * (&wr[1] * &wr[1] + &wr[2] * &wr[2]);
    let h = (&sl * &hl + &sr * &hr) * &inv;
    let c = (cf(G1) * (&h - 0.5f32 * (&u * &u + &v * &v))).maximum(1e-30f32).sqrt();
    let a = &u * nx + &v * nr;
    let eps = eps_fix * &c;
    let fix = |l: &F| -> F {
        let al = l.abs();
        select(&al.ge(&eps), &al, 0.5f32 * (l * l / &eps + &eps))
    };
    let (e1, e2, e3) = (fix(&a), fix(&(&a + &c)), fix(&(&a - &c)));
    let d1 = (&e2 - &e1) / &c;
    let d2 = (&e2 + &e3 - 2.0f32 * &e1) / (2.0f32 * &c * &c);
    let am = flux_jacobian_h(&u, &v, &h, nx, nr);
    let b1 = am.sub_diag(&a);
    let b2 = am.sub_diag(&(&a + &c));
    let prod = b1.mm(&b2);
    b1.scale(&d1).add(&prod.scale(&d2)).add_diag(&e1)
}

/// Dissipation matrix of the face between the states `l` and `r`: Roe-type `|A|` blended with the scalar Rusanov
/// `lambda I`, `lambda = max(|u_n| + c)` of the two states, by the shock weight `t` (`(1 - t) |A| + t lambda I`); `roe = false`:
/// `lambda I` alone.
fn dissipation_f(l: &V4, r: &V4, nx: &F, nr: &F, p: &ImplicitParams) -> Blk {
    let lam = spectral_f(l, nx, nr).maximum(spectral_f(r, nx, nr));
    if p.roe {
        let t = shock_weight_f(&l[3], &r[3]);
        abs_jacobian_f(l, r, nx, nr, cf(p.eps_fix)).scale(&(1.0f32 - &t)).add_diag(&(&t * &lam))
    } else {
        Blk::zero().add_diag(&lam)
    }
}

/// [`dissipation_f`] with the preconditioning: the Roe-type part is `P |P^-1 A|` ([`abs_jacobian_pc_f`]); the Rusanov part (shock blend,
/// `lambda = max(|u_n| + c)` of the two states) and `roe = false` are those of the right-hand side's `lambda`.
fn dissipation_pc_f(l: &V4, r: &V4, nx: &F, nr: &F, p: &ImplicitParams, lm: &LmGpu) -> Blk {
    let lam = spectral_f(l, nx, nr).maximum(spectral_f(r, nx, nr));
    if p.roe {
        let t = shock_weight_f(&l[3], &r[3]);
        abs_jacobian_pc_f(l, r, nx, nr, cf(p.eps_fix), lm).scale(&(1.0f32 - &t)).add_diag(&(&t * &lam))
    } else {
        Blk::zero().add_diag(&lam)
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// ghost-state Jacobians (f32 copies of body.rs's ghost states: only the left-hand side uses them)
// ---------------------------------------------------------------------------------------------------------------------

/// [`characteristic_ghost`] in f32 (the same Riemann-invariant far field).
fn characteristic_ghost_f(fs: &[f64; 4], interior: &V4, nx: &F, nr: &F) -> V4 {
    let ci = (cf(GAMMA) * &interior[3] / &interior[0]).sqrt();
    let uni = &interior[1] * nx + &interior[2] * nr;
    let m = &uni / &ci;
    let ce = (GAMMA * fs[3] / fs[0]).sqrt();
    let une = cf(fs[1]) * nx + cf(fs[2]) * nr;
    let r_out = &uni + 2.0f32 * &ci / cf(G1);
    let r_in = &une - cf(2.0 * ce / G1);
    let un_b = 0.5f32 * (&r_out + &r_in);
    let c_b = (cf(0.25 * G1) * (&r_out - &r_in)).maximum(1e-8f32);
    let from_interior = un_b.ge(0.0f32);
    let rf = [select(&from_interior, &interior[0], cf(fs[0])), select(&from_interior, &interior[1], cf(fs[1])), select(&from_interior, &interior[2], cf(fs[2]))];
    let un_ref = select(&from_interior, &uni, &une);
    let c_ref = select(&from_interior, &ci, cf(ce));
    let q = &c_b / &c_ref;
    let q2 = &q * &q;
    let rho = &rf[0] * (&q2 * &q2 * &q);
    let dn = &un_b - &un_ref;
    let riemann = [rho.clone(), &rf[1] + &dn * nx, &rf[2] + &dn * nr, &rho * &c_b * &c_b / cf(GAMMA)];
    let super_in = m.le(-1.0f32);
    let super_out = m.ge(1.0f32);
    std::array::from_fn(|k| select(&super_in, cf(fs[k]), select(&super_out, &interior[k], &riemann[k])))
}

/// [`pressure_outflow_ghost`] in f32.
fn pressure_outflow_ghost_f(p_back: f64, interior: &V4, nx: &F, nr: &F) -> V4 {
    let ci = (cf(GAMMA) * &interior[3] / &interior[0]).sqrt();
    let m = (&interior[1] * nx + &interior[2] * nr) / &ci;
    [interior[0].clone(), interior[1].clone(), interior[2].clone(), select(&m.ge(1.0f32), &interior[3], cf(p_back))]
}

/// `T = dU_ghost / dU_interior` of a boundary's first ghost layer.
#[allow(clippy::large_enum_variant)]
enum Tj {
    /// Fixed free stream.
    Zero,
    /// Zero-gradient extrapolation.
    Eye,
    Mat(Blk),
}

/// `m T`.
fn times_t(m: &Blk, t: &Tj) -> Option<Blk> {
    match t {
        Tj::Zero => None,
        Tj::Eye => Some(m.clone()),
        Tj::Mat(t) => Some(m.mm(t)),
    }
}

/// Reflection matrix of the conserved variables about the wall normal `(n_x, n_r)`: momentum mirrored, scalars kept.
fn reflection_f(nx: &F, nr: &F) -> Blk {
    let (nxx, nxr, nrr) = (nx * nx, nx * nr, nr * nr);
    let a = E::A;
    Blk([
        E::O,
        E::Z,
        E::Z,
        E::Z,
        E::Z,
        a(1.0f32 - 2.0f32 * &nxx),
        a(-2.0f32 * &nxr),
        E::Z,
        E::Z,
        a(-2.0f32 * &nxr),
        a(1.0f32 - 2.0f32 * &nrr),
        E::Z,
        E::Z,
        E::Z,
        E::Z,
        E::O,
    ])
}

/// The geometry of one domain boundary on its edge cells (f32): unit normal of the boundary face and the outward normal.
struct EdgeGeo {
    nx: F,
    nr: F,
    onx: F,
    onr: F,
}

/// Uploaded f32 geometry of one direction: unit normal, length and flux weight `len r_mid` of its faces, and the masks of the first
/// and last cell of every line.
struct LDir {
    nx: F,
    nr: F,
    len: F,
    w: F,
    first: Array<bool>,
    last: Array<bool>,
    /// Zeros and ones of the face shape.
    zero: F,
    one: F,
}

/// The f32 geometry of the left-hand side, uploaded once from the grid (the arrays of `Geo` are of the precision `S`).
struct LGeo {
    /// i faces `[ni + 1, nj]` and j faces `[ni, nj + 1]`.
    dir: [LDir; 2],
    /// Cell volume `V`, area `A` and `A / V`.
    vol: F,
    area: F,
    src: F,
    /// Zeros and ones `[ni, nj]`.
    zero: F,
    one: F,
    /// `[imin, imax, jmin, jmax]`: `[1, nj]` or `[ni, 1]` arrays.
    edge: [EdgeGeo; 4],
}

impl LGeo {
    fn new(ctx: &Context, grid: &Grid) -> Result<LGeo> {
        let (ni, nj) = (grid.ni, grid.nj);
        let face = |f: &[crate::grid::Face], shape: [usize; 2], g: &dyn Fn(&crate::grid::Face) -> f64| up_lhs(ctx, &f.iter().map(g).collect::<Vec<_>>(), shape);
        let mask = |shape: [usize; 2], f: &dyn Fn(usize, usize) -> bool| {
            let v: Vec<bool> = (0..shape[1]).flat_map(|j| (0..shape[0]).map(move |i| (i, j))).map(|(i, j)| f(i, j)).collect();
            Array::from_slice_on(ctx, &v, shape)
        };
        let dir_i = LDir {
            nx: face(&grid.iface, [ni + 1, nj], &|f| f.nx)?,
            nr: face(&grid.iface, [ni + 1, nj], &|f| f.nr)?,
            len: face(&grid.iface, [ni + 1, nj], &|f| f.len)?,
            w: face(&grid.iface, [ni + 1, nj], &|f| f.w())?,
            first: mask([ni, nj], &|i, _| i == 0)?,
            last: mask([ni, nj], &|i, _| i + 1 == ni)?,
            zero: up_lhs(ctx, &vec![0.0; (ni + 1) * nj], [ni + 1, nj])?,
            one: up_lhs(ctx, &vec![1.0; (ni + 1) * nj], [ni + 1, nj])?,
        };
        let dir_j = LDir {
            nx: face(&grid.jface, [ni, nj + 1], &|f| f.nx)?,
            nr: face(&grid.jface, [ni, nj + 1], &|f| f.nr)?,
            len: face(&grid.jface, [ni, nj + 1], &|f| f.len)?,
            w: face(&grid.jface, [ni, nj + 1], &|f| f.w())?,
            first: mask([ni, nj], &|_, j| j == 0)?,
            last: mask([ni, nj], &|_, j| j + 1 == nj)?,
            zero: up_lhs(ctx, &vec![0.0; ni * (nj + 1)], [ni, nj + 1])?,
            one: up_lhs(ctx, &vec![1.0; ni * (nj + 1)], [ni, nj + 1])?,
        };
        let edge = |side: usize| -> Result<EdgeGeo> {
            let (faces, shape, outward): (Vec<crate::grid::Face>, [usize; 2], f64) = match side {
                0 => ((0..nj).map(|j| grid.iface[j * (ni + 1)]).collect(), [1, nj], -1.0),
                1 => ((0..nj).map(|j| grid.iface[j * (ni + 1) + ni]).collect(), [1, nj], 1.0),
                2 => ((0..ni).map(|i| grid.jface[i]).collect(), [ni, 1], -1.0),
                _ => ((0..ni).map(|i| grid.jface[nj * ni + i]).collect(), [ni, 1], 1.0),
            };
            let get = |g: &dyn Fn(&crate::grid::Face) -> f64| up_lhs(ctx, &faces.iter().map(g).collect::<Vec<_>>(), shape);
            Ok(EdgeGeo { nx: get(&|f| f.nx)?, nr: get(&|f| f.nr)?, onx: get(&|f| outward * f.nx)?, onr: get(&|f| outward * f.nr)? })
        };
        Ok(LGeo {
            dir: [dir_i, dir_j],
            vol: up_lhs(ctx, &grid.vol, [ni, nj])?,
            area: up_lhs(ctx, &grid.area, [ni, nj])?,
            src: up_lhs(ctx, &grid.area.iter().zip(&grid.vol).map(|(a, v)| a / v).collect::<Vec<_>>(), [ni, nj])?,
            zero: up_lhs(ctx, &vec![0.0; ni * nj], [ni, nj])?,
            one: up_lhs(ctx, &vec![1.0; ni * nj], [ni, nj])?,
            edge: [edge(0)?, edge(1)?, edge(2)?, edge(3)?],
        })
    }
}

/// `w` reflected about a wall of unit normal `(n_x, n_r)`: `v - 2 (v.n) n` (f32).
fn reflect_f(w: &V4, nx: &F, nr: &F) -> V4 {
    let un = &w[1] * nx + &w[2] * nr;
    [w[0].clone(), &w[1] - 2.0f32 * &un * nx, &w[2] - 2.0f32 * &un * nr, w[3].clone()]
}

/// The solid block seen by the implicit left-hand side (`set_block`): masks (`u32`, 1 = true) and the start of every line.
struct LBlockGeo {
    /// Solid cells `[ni, nj]`.
    solid: Array<bool>,
    /// Fluid cells (`!solid`).
    fluid: Array<bool>,
    /// First fluid cell of every i line (`row_start(j)`, `ni` for a solid row; `nj` elements) and j line (`col_start(i)`; `ni` elements).
    start: [Array<u32>; 2],
    /// First fluid cell of a line, at the domain boundary (`k0 = 0`: ghost-state Jacobian), behind the solid (`k0 > 0`: wall, reflection),
    /// and either.
    first_bc: [Array<bool>; 2],
    first_refl: [Array<bool>; 2],
    first: [Array<bool>; 2],
    /// Base-wall faces `[ni + 1, nj]`: `i = i_end`, rows `j_sting..j_end`.
    base: Array<bool>,
    /// Nozzle exit faces `[ni + 1, nj]` (`i = i_end`, rows `j < nozzle.j_end`); `None` without a nozzle.
    jet: Option<Array<bool>>,
    /// The jet rows of the base-face flux stencils: `[2, j_end - j_sting]` (faces `i_end`, `i_end + 1`) and `[1, j_end - j_sting]` (face `i_end`).
    jet_rows: Option<[Array<bool>; 2]>,
}

impl LBlockGeo {
    fn new(ctx: &Context, ni: usize, nj: usize, b: crate::grid::Block, nozzle: Option<crate::solver::Nozzle>) -> Result<LBlockGeo> {
        let mask = |shape: [usize; 2], f: &dyn Fn(usize, usize) -> bool| {
            let v: Vec<bool> = (0..shape[1]).flat_map(|j| (0..shape[0]).map(move |i| (i, j))).map(|(i, j)| f(i, j)).collect();
            Array::from_slice_on(ctx, &v, shape)
        };
        // first fluid cell of the line through (i, j) of direction `dir`, if the line has one
        let k0 = |dir: usize, i: usize, j: usize| if dir == 0 { b.row_start(j) } else { Some(b.col_start(i)) };
        let at = |dir: usize, i: usize, j: usize| k0(dir, i, j) == Some(if dir == 0 { i } else { j });
        let first_k0 = |dir: usize, i: usize, j: usize| k0(dir, i, j).filter(|_| at(dir, i, j));
        let start = |dir: usize| {
            let v: Vec<u32> = if dir == 0 { (0..nj).map(|j| b.row_start(j).unwrap_or(ni) as u32).collect() } else { (0..ni).map(|i| b.col_start(i) as u32).collect() };
            Array::from_slice_on(ctx, &v, v.len())
        };
        Ok(LBlockGeo {
            solid: mask([ni, nj], &|i, j| b.solid(i, j))?,
            fluid: mask([ni, nj], &|i, j| !b.solid(i, j))?,
            start: [start(0)?, start(1)?],
            first_bc: [mask([ni, nj], &|i, j| first_k0(0, i, j) == Some(0))?, mask([ni, nj], &|i, j| first_k0(1, i, j) == Some(0))?],
            first_refl: [
                mask([ni, nj], &|i, j| first_k0(0, i, j).is_some_and(|k| k > 0) && !nozzle.is_some_and(|n| i == b.i_end && j < n.j_end))?,
                mask([ni, nj], &|i, j| first_k0(1, i, j).is_some_and(|k| k > 0))?,
            ],
            first: [mask([ni, nj], &|i, j| at(0, i, j))?, mask([ni, nj], &|i, j| at(1, i, j))?],
            base: mask([ni + 1, nj], &|f, j| f == b.i_end && j >= b.j_sting && j < b.j_end)?,
            jet: nozzle.map(|n| mask([ni + 1, nj], &|f, j| f == b.i_end && j < n.j_end)).transpose()?,
            jet_rows: nozzle
                .map(|n| -> Result<[Array<bool>; 2]> { Ok([mask([2, b.j_end - b.j_sting], &|_, j| j < n.j_end)?, mask([1, b.j_end - b.j_sting], &|_, j| j < n.j_end)?]) })
                .transpose()?,
        })
    }
}

/// The blocks `lo`, `d`, `up` of `I - dt J_dir` of every cell (`[ni, nj]` arrays, evaluated).
struct Factor {
    lo: [F; 16],
    d: [F; 16],
    up: [F; 16],
}

impl LazySolver {
    /// The f32 view of the state `a` (the left-hand side runs in f32; see the module comment).
    fn lhs_rec(r: &Rec) -> V4 {
        std::array::from_fn(|k| lhs(&r[k]))
    }

    /// Number of cells along the lines of direction `dir` (0: i lines, 1: j lines).
    fn line_len(&self, dir: usize) -> usize {
        if dir == 0 {
            self.ni
        } else {
            self.nj
        }
    }

    /// `T = dU_ghost / dU_interior` of the first ghost layer of boundary `side`, for the interior (edge) state `edge`:
    /// exact for the free stream (0), extrapolation (`I`) and the slip wall (reflection); for the characteristic and pressure-outflow
    /// far fields the central finite difference of the ghost state with the relative step `1e-3` of the hand-written kernels (the
    /// CPU's `1e-7` is below f32 resolution): column `q` is `(U_g(U + h e_q) - U_g(U - h e_q)) / 2 h`, `h = 1e-3 (1 + |U_q|)`.
    /// `ImplicitParams::implicit_bc = false` freezes the ghost states (`T = 0`).
    fn ghost_jacobian(&self, side: usize, edge: &V4, p: &ImplicitParams) -> Tj {
        if !p.implicit_bc {
            return Tj::Zero;
        }
        let e = &self.lg.edge[side];
        let fs = self.cfg.freestream;
        let ghost = |w: &V4| -> V4 {
            match self.cfg.bc[side] {
                Bc::Characteristic => characteristic_ghost_f(&fs, w, &e.onx, &e.onr),
                _ => pressure_outflow_ghost_f(fs[3], w, &e.onx, &e.onr),
            }
        };
        match self.cfg.bc[side] {
            Bc::Freestream => Tj::Zero,
            Bc::Extrapolate => Tj::Eye,
            Bc::SlipWall => Tj::Mat(self.wall_jacobian(side, e)),
            Bc::Characteristic | Bc::PressureOutflow => {
                let u0 = prim_to_cons_f(edge);
                let mut entries: [E; 16] = std::array::from_fn(|_| E::Z);
                for q in 0..4 {
                    let h = 1e-3f32 * (1.0f32 + u0[q].abs());
                    let ghost_at = |sign: f32| -> V4 {
                        let mut uq = u0.clone();
                        uq[q] = &u0[q] + sign * &h;
                        prim_to_cons_f(&ghost(&cons_to_prim_f(&uq)))
                    };
                    let (gp, gm) = (ghost_at(1.0), ghost_at(-1.0));
                    for r in 0..4 {
                        entries[4 * r + q] = E::A((&gp[r] - &gm[r]) / (2.0f32 * &h));
                    }
                }
                Tj::Mat(Blk(entries))
            }
        }
    }

    /// Local pseudo-time step from the cell primitives `wk` (f32): `dt = cfl A / (L_i + L_j)` (`DtMode::Sum`) or
    /// `cfl A / (2 min(L_i, L_j))` (`DtMode::MinDirection`), `L_d` = half the sum of `(|u.n| + c) len` over the two faces of direction `d`.
    /// `cfl` is the one-element array of the loop state.
    fn pseudo_dt(&self, wk: &V4, cfl: &F, mode: DtMode, xij: Option<(F, F)>, lm: Option<&LmGpu>) -> F {
        let g = &self.lg;
        let c2 = cf(GAMMA) * &wk[3] / &wk[0];
        let cs = c2.sqrt();
        // preconditioned: `U_r^2 / c^2 = b`, `|u'| + c'` with `u' = u_n (1 + b) / 2`, `c' = sqrt(u_n^2 (1 - b)^2 + 4 b c^2) / 2`
        let b = lm.map(|l| &l.ur2(&(&wk[1] * &wk[1] + &wk[2] * &wk[2]), &c2) / &c2);
        let lam = |dir: usize, upper: bool| -> F {
            let d = &g.dir[dir];
            let n = self.line_len(dir);
            let r = if upper { 1..n + 1 } else { 0..n };
            let un = &wk[1] * d.nx.slice(dir, r.clone()) + &wk[2] * d.nr.slice(dir, r.clone());
            let rad = match &b {
                Some(b) => {
                    let omb = 1.0f32 - b;
                    (0.5f32 * &un * (1.0f32 + b)).abs() + 0.5f32 * (&un * &un * &omb * &omb + 4.0f32 * b * &c2).sqrt()
                }
                None => un.abs() + &cs,
            };
            rad * d.len.slice(dir, r)
        };
        let li = 0.5f32 * (lam(0, false) + lam(0, true));
        let lj = 0.5f32 * (lam(1, false) + lam(1, true));
        // a viscous run adds the diffusive limits of the two directions (`diffusion_rates`)
        let (li, lj) = match xij {
            Some((xi, xj)) => (li + xi, lj + xj),
            None => (li, lj),
        };
        match mode {
            DtMode::Sum => cfl * &g.area / (&li + &lj),
            DtMode::MinDirection => cfl * &g.area / (2.0f32 * li.minimum(&lj)),
        }
    }

    /// Blocks `(lo, d, up)` of `I - dt J_dir` for every cell (`assemble`, `cell_blocks`). With the face matrices
    /// `FL = A(U_L) + D`, `FR = A(U_R) - D` of every face (`D` the dissipation matrix of the face, `L` / `R` its lower / upper
    /// state, ghost states at the ends of a line, `A` evaluated with the face normal) and the cell coefficients `s_l = dt w_l / (2 V)`,
    /// `s_u = dt w_u / (2 V)` (`w` the flux weights of the lower / upper face):
    ///
    /// ```text
    /// lo = -s_l FL_lower,   d = I + s_u FL_upper - s_l FR_lower (+ axisymmetric source, j lines),   up = s_u FR_upper
    /// ```
    ///
    /// The source `S = (0, 0, p A / V, 0)` adds `-dt dS/dU` to rows 2 of `d`: with `c = -dt (A / V) (g - 1)`,
    /// `(c q^2 / 2, -c u, -c v, c)` in columns 0..3. The first / last cell of a line folds `lo T` / `up T` into `d` (`T` from
    /// [`LazySolver::ghost_jacobian`]) and drops `lo` / `up`. `lines = false`: point-implicit, `lo = up = 0`.
    fn factor(&self, wf: &V4, wk: &V4, dt: &F, p: &ImplicitParams, dir: usize, vf: Option<&VFlow>, lm: Option<&LmGpu>, pk: Option<&[F; 16]>) -> Factor {
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.lg;
        let dg = &g.dir[dir];
        let n = self.line_len(dir);

        // face matrices: the states below / above every face of the direction, from the padded primitives (two ghost layers)
        let (l, r): (V4, V4) = if dir == 0 {
            let wi: V4 = std::array::from_fn(|k| wf[k].slice(1, 2..nj + 2));
            (std::array::from_fn(|k| wi[k].slice(0, 1..ni + 2)), std::array::from_fn(|k| wi[k].slice(0, 2..ni + 3)))
        } else {
            let wj: V4 = std::array::from_fn(|k| wf[k].slice(0, 2..ni + 2));
            (std::array::from_fn(|k| wj[k].slice(1, 1..nj + 2)), std::array::from_fn(|k| wj[k].slice(1, 2..nj + 3)))
        };
        // the base wall: the state below the face `i_end` of a row `j_sting..j_end` is the reflection of the first fluid cell about the face
        // (not the padded state there, which is solid or a j-direction block ghost)
        let l: V4 = match (&self.lb, dir) {
            (Some(lb), 0) => {
                let refl = self.base_wall_ghost_f(&r, &dg.nx, &dg.nr);
                let l: V4 = std::array::from_fn(|k| select(&lb.base, &refl[k], &l[k]));
                // the nozzle exit: the fixed jet state below the faces of the jet rows
                match (&lb.jet, self.nozzle) {
                    (Some(jet), Some(n)) => std::array::from_fn(|k| select(jet, cf(n.state[k]), &l[k])),
                    _ => l,
                }
            }
            _ => l,
        };
        let (first_bc, first_any) = match &self.lb {
            Some(lb) => (&lb.first_bc[dir], &lb.first[dir]),
            None => (&dg.first, &dg.first),
        };
        let dis = match lm {
            Some(lmc) => dissipation_pc_f(&l, &r, &dg.nx, &dg.nr, p, lmc),
            None => dissipation_f(&l, &r, &dg.nx, &dg.nr, p),
        };
        // a viscous run adds the diffusive spectral radius `2 lam I` to the dissipation matrix of every face
        let dis = match vf {
            Some(v) => dis.add_diag(&self.two_lambda(v, dir, &l[0], &r[0])),
            None => dis,
        };
        let fl = flux_jacobian_f(&l, &dg.nx, &dg.nr).add(&dis);
        let fr = flux_jacobian_f(&r, &dg.nx, &dg.nr).sub(&dis);
        // evaluate the face matrices once: the cell blocks read each of them at two cells
        let [fl, fr]: [[F; 16]; 2] = [fl.dense(&dg.zero, &dg.one), fr.dense(&dg.zero, &dg.one)].cache();
        let (fl, fr) = (Blk::of(&fl), Blk::of(&fr));

        // cell blocks
        let s = 0.5f32 * dt / &g.vol;
        let (s_l, s_u) = (&s * dg.w.slice(dir, 0..n), &s * dg.w.slice(dir, 1..n + 1));
        let mut d = pk.map_or_else(Blk::eye, Blk::of).add(&fl.slice(dir, 1..n + 1).scale(&s_u)).sub(&fr.slice(dir, 0..n).scale(&s_l));
        if dir == 1 {
            let c = -(dt * &g.src) * cf(G1);
            let src = [0.5f32 * &c * (&wk[1] * &wk[1] + &wk[2] * &wk[2]), -(&c * &wk[1]), -(&c * &wk[2]), c.clone()];
            for (k, x) in src.into_iter().enumerate() {
                d.0[8 + k] = d.0[8 + k].add(&E::A(x));
            }
        }
        let (mut lo, mut up) = if p.lines { (fl.slice(dir, 0..n).scale(&(-&s_l)), fr.slice(dir, 1..n + 1).scale(&s_u)) } else { (Blk::zero(), Blk::zero()) };

        // ends of the lines: fold `lo T` / `up T` into `d`
        let (lo_side, hi_side) = if dir == 0 { (0, 1) } else { (2, 3) };
        if p.lines {
            let edge = |at: std::ops::Range<usize>| -> V4 { std::array::from_fn(|k| wk[k].slice(dir, at.clone())) };
            let (t_lo, t_hi) = (self.ghost_jacobian(lo_side, &edge(0..1), p), self.ghost_jacobian(hi_side, &edge(n - 1..n), p));
            let folds = [times_t(&lo.slice(dir, 0..1), &t_lo), times_t(&up.slice(dir, n - 1..n), &t_hi)];
            let (z, o) = (g.zero.slice(dir, 0..1), g.one.slice(dir, 0..1));
            // the first fluid cell of a line that starts behind the solid (`k0 > 0`) sees the wall: `T` = reflection about its lower face
            let refl_fold = self.lb.as_ref().map(|_| lo.mm(&reflection_f(&dg.nx.slice(dir, 0..n), &dg.nr.slice(dir, 0..n))));
            // viscous run: the first fluid cell behind a no-slip wall sees `T = diag(1, -1, -1, 1)` instead
            let noslip_fold = self.visc.as_ref().and_then(|v| v.fold.as_ref()).map(|_| lo.noslip_cols());
            let present: [[bool; 16]; 2] = std::array::from_fn(|s| std::array::from_fn(|k| folds[s].as_ref().is_some_and(|b| !matches!(b.0[k], E::Z))));
            let dense: [[F; 16]; 2] = std::array::from_fn(|s| folds[s].as_ref().map_or_else(|| std::array::from_fn(|_| z.clone()), |b| b.dense(&z, &o)));
            let dense = dense.cache();
            let spread = |a: &F, low: bool| -> F {
                match (dir, low) {
                    (0, true) => a.pad([(0, n - 1)], PadMode::Edge),
                    (0, false) => a.pad([(n - 1, 0)], PadMode::Edge),
                    (_, true) => a.pad([(0, 0), (0, n - 1)], PadMode::Edge),
                    (_, false) => a.pad([(0, 0), (n - 1, 0)], PadMode::Edge),
                }
            };
            for k in 0..16 {
                for (s, mask) in [(0, first_bc), (1, &dg.last)] {
                    if present[s][k] {
                        let add = select(mask, spread(&dense[s][k], s == 0), 0.0f32);
                        d.0[k] = d.0[k].add(&E::A(add));
                    }
                }
                if let (Some(lb), Some(rf)) = (&self.lb, &refl_fold) {
                    if let E::A(x) = &rf.0[k] {
                        let slip = self.visc.as_ref().and_then(|v| v.fold.as_ref()).map_or(&lb.first_refl[dir], |f| &f[dir][0]);
                        d.0[k] = d.0[k].add(&E::A(select(slip, x, 0.0f32)));
                    }
                }
                if let (Some(f), Some(nf)) = (self.visc.as_ref().and_then(|v| v.fold.as_ref()), &noslip_fold) {
                    if let E::A(x) = &nf.0[k] {
                        d.0[k] = d.0[k].add(&E::A(select(&f[dir][1], x, 0.0f32)));
                    }
                }
                if let (E::A(a), E::A(b)) = (&lo.0[k], &up.0[k]) {
                    lo.0[k] = E::A(select(first_any, 0.0f32, a));
                    up.0[k] = E::A(select(&dg.last, 0.0f32, b));
                }
            }
        }

        // solid cells: `d = I`, `lo = up = 0` (selects, not multiplies: nothing computed inside the solid reaches the blocks)
        if let Some(lb) = &self.lb {
            let mask = |b: &Blk, diag: bool| -> Blk {
                Blk(std::array::from_fn(|k| match &b.0[k] {
                    E::A(a) => E::A(select(&lb.solid, if diag && k % 5 == 0 { 1.0f32 } else { 0.0f32 }, a)),
                    e => e.clone(),
                }))
            };
            (lo, d, up) = (mask(&lo, false), mask(&d, true), mask(&up, false));
        }
        let (z, o) = (&g.zero, &g.one);
        let [lo, d, up] = [lo.dense(z, o), d.dense(z, o), up.dense(z, o)].cache();
        Factor { lo, d, up }
    }

    /// `x <- (I - dt J_dir)^{-1} x` for every line of direction `dir` (block-tridiagonal solve, `solve_lines`); the sixteen block entries
    /// and the four components are passed as parts (no stacking).
    fn solve_lines(&self, fac: &Factor, dir: usize, x: &V4) -> V4 {
        let lo: [&F; 16] = std::array::from_fn(|k| &fac.lo[k]);
        let d: [&F; 16] = std::array::from_fn(|k| &fac.d[k]);
        let up: [&F; 16] = std::array::from_fn(|k| &fac.up[k]);
        let rhs: [&F; 4] = std::array::from_fn(|k| &x[k]);
        match &self.lb {
            // lines start at their first fluid cell; the cells before it (and whole solid lines) get 0
            Some(lb) => luma::solve_block_tridiagonal_parts_from(dir, lo, d, up, rhs, &lb.start[dir]),
            None => luma::solve_block_tridiagonal_parts(dir, lo, d, up, rhs),
        }
    }

    /// The approximate factorization `(I - dt J_j)^{-1} (I - dt J_i)^{-1} rhs`; preconditioned (`pm` = `P`):
    /// `(P - dt J_j)^{-1} P (P - dt J_i)^{-1} rhs`.
    fn approx_factor(&self, fi: &Factor, fj: &Factor, rhs: &V4, pm: Option<&[F; 16]>) -> V4 {
        let x = self.solve_lines(fi, 0, rhs);
        match pm {
            Some(pm) => self.solve_lines(fj, 1, &mv(pm, &x).cache()),
            None => self.solve_lines(fj, 1, &x),
        }
    }

    /// `(I - dt J_dir) x` (`apply_factor`): `d x + lo x_{k-1} + up x_{k+1}`, the missing neighbours at the ends of a line being 0 (their
    /// blocks are zero there).
    fn apply_factor(&self, fac: &Factor, dir: usize, x: &V4) -> V4 {
        let (lower, upper): ([isize; 2], [isize; 2]) = if dir == 0 { ([1, 0], [-1, 0]) } else { ([0, 1], [0, -1]) };
        let xm: V4 = std::array::from_fn(|k| x[k].shift_fill(lower, 0.0));
        let xp: V4 = std::array::from_fn(|k| x[k].shift_fill(upper, 0.0));
        let (dx, lx, ux) = (mv(&fac.d, x), mv(&fac.lo, &xm), mv(&fac.up, &xp));
        std::array::from_fn(|q| &dx[q] + &lx[q] + &ux[q])
    }

    /// Positivity damping: `alpha` is the first of `1, 1/2, .., 1/128` for which `U + alpha dU` has `rho > 0`, `p > 0` and four
    /// finite components (`is_finite`), `0` if there is none. Decided in f32 from the f32 view of `U`.
    fn damping(&self, u: &V4, du: &V4) -> F {
        let ok = |a: f32| -> Array<bool> {
            let q: V4 = std::array::from_fn(|k| &u[k] + a * &du[k]);
            let w = cons_to_prim_f(&q);
            let finite = q.iter().map(|x| x.is_finite()).reduce(|a, b| a & b).expect("the conserved state has four components");
            w[0].gt(0.0f32) & w[3].gt(0.0f32) & finite
        };
        let last = 1.0f32 / 128.0;
        let mut alpha = select(&ok(last), last, 0.0f32);
        for k in (0..7).rev() {
            let a = 0.5f32.powi(k);
            alpha = select(&ok(a), a, &alpha);
        }
        alpha
    }

    /// One implicit iteration of the loop state `s` (`implicit::step`): the new state and the density residual of `R` at the start of
    /// the iteration (the residual of the step that produced it is `s[4]`), the CFL ramp advanced: `cfl <- min(growth cfl, cfl_max)`,
    /// `cfl` being `s[5]` (it starts at `cfl0` at [`LazySolver::restart_ramp`]).
    ///
    /// ```text
    /// X = AF^{-1} (dt R),  r = dt R - (A_i + A_j - I) X,  X += AF^{-1} r      (sub_iters - 1 times),   U += alpha X
    /// ```
    ///
    /// with `AF = (I - dt J_i)(I - dt J_j)` (factored) and `A_d = I - dt J_d`.
    fn implicit_step(&self, s: &State, p: &ImplicitParams, lim: Limiter) -> State {
        let (ni, nj) = (self.ni, self.nj);
        let u = unpack(s);
        let (prim, w) = self.padded(&u);
        let st = self.stage_from(prim, w, self.sst().then(|| &s[8]), lim, false);
        let (r0, w) = (&st.res, &st.w);
        let rn = self.residual_norm(&r0[0]);

        // the f32 left-hand side: primitives with ghosts, pseudo-time step, factors
        let wf: V4 = Self::lhs_rec(w).cache();
        let wk: V4 = std::array::from_fn(|k| wf[k].slice(0, 2..ni + 2).slice(1, 2..nj + 2));
        let cfl = lhs(&s[5]);
        let xij = st.vf.as_ref().map(|v| {
            let (xi, xj) = self.diffusion_rates(v, &st.p[0]);
            (lhs(&xi), lhs(&xj))
        });
        // low-Mach preconditioning (`ImplicitParams::low_mach_precond`): `P` of every cell, identity in solid cells
        let lm = LmGpu::of(p, &self.cfg.freestream);
        let pk: Option<[F; 16]> = lm.as_ref().map(|l| precond_p(&wk, l, self.lb.as_ref().map(|lb| &lb.solid)).cache());
        let dt = self.pseudo_dt(&wk, &cfl, p.dt_mode, xij, lm.as_ref());
        // solid cells: dt = 0 (as on the CPU), so that `dt R`, the right-hand side of the solves, is zero there
        let dt = match &self.lb {
            Some(lb) => select(&lb.solid, 0.0f32, &dt),
            None => dt,
        };
        let (fi, fj) = (
            self.factor(&wf, &wk, &dt, p, 0, st.vf.as_ref(), lm.as_ref(), pk.as_ref()),
            self.factor(&wf, &wk, &dt, p, 1, st.vf.as_ref(), lm.as_ref(), pk.as_ref()),
        );

        // right-hand side and defect correction
        let b: V4 = std::array::from_fn(|k| &lhs(&r0[k]) * &dt);
        let b = b.cache();
        let mut du = self.approx_factor(&fi, &fj, &b, pk.as_ref());
        for _ in 1..p.sub_iters.max(1) {
            let (ai, aj) = (self.apply_factor(&fi, 0, &du), self.apply_factor(&fj, 1, &du));
            // the defect operator `A_i + A_j - P` with the preconditioned factors
            let pdu = pk.as_ref().map_or_else(|| du.clone(), |pm| mv(pm, &du));
            let r: V4 = std::array::from_fn(|k| &b[k] - (&ai[k] + &aj[k] - &pdu[k]));
            let corr = self.approx_factor(&fi, &fj, &r.cache(), pk.as_ref());
            du = std::array::from_fn(|k| &du[k] + &corr[k]);
        }

        // update with positivity damping
        let alpha = self.damping(&Self::lhs_rec(&u), &du);
        let moved = match &self.lb {
            Some(lb) => alpha.gt(0.0f32) & &lb.fluid,
            None => alpha.gt(0.0f32),
        };
        let alpha = from_lhs(&alpha);
        let un: Rec = std::array::from_fn(|k| select(&moved, &u[k] + &alpha * from_lhs(&du[k]), &u[k]));
        let cfl_next = (&s[5] * c(p.growth)).minimum(c(p.cfl_max));
        pack(un, rn, cfl_next, self.turb_next(s, &st, &from_lhs(&dt)))
    }

    /// One iteration of the configured scheme from the loop state.
    fn step(&self, s: &State, lim: Limiter) -> State {
        match self.scheme {
            Scheme::Implicit(p) => self.implicit_step(s, &p, lim),
            _ => {
                let (u, rn, st) = self.rk3(&unpack(s), self.sst().then(|| &s[8]), lim);
                pack(u, rn, &s[5] * c(1.0), self.turb_next(s, &st, st.dt.as_ref().expect("rk3 computes dt")))
            }
        }
    }

    /// CFL of the first iteration after a restart of the ramp: `min(cfl0, cfl_max)` for the implicit scheme (the CFL of the explicit
    /// one is the constant `Config::cfl`).
    fn cfl_start(&self) -> f64 {
        match self.scheme {
            Scheme::Implicit(p) => p.cfl0.min(p.cfl_max),
            _ => self.cfg.cfl,
        }
    }

    /// Selects the scheme: `Scheme::Explicit` (SSP-RK3) or `Scheme::Implicit`; restarts the CFL ramp. The smoothed and multigrid
    /// schemes are not covered.
    pub fn set_scheme(&mut self, scheme: Scheme) -> Result<()> {
        if !matches!(scheme, Scheme::Explicit | Scheme::Implicit(_)) {
            return Err(luma::LumaError::Unsupported("the lazy solver covers Scheme::Explicit and Scheme::Implicit only".into()));
        }
        self.scheme = scheme;
        self.restart_ramp()?;
        Ok(())
    }

    /// Restarts the CFL ramp of the implicit scheme at `cfl0` (call at the start of a run and after a change of the free stream).
    pub fn restart_ramp(&mut self) -> Result<()> {
        let cfl = up(&self.ctx, &[self.cfl_start()], 1)?;
        if let Some(s) = &mut self.state {
            s[5] = cfl;
        }
        Ok(())
    }
}
