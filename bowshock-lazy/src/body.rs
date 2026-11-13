// The explicit solver in array ops. This file is `include!`d twice by `lib.rs`, once per precision, into a module that
// defines `S` (the element type: `f32` or `Df32`), `c(f64) -> S` (a constant rounded from f64), `up(..)` (host f64 data to
// an `Array<S>`), `to_f64(..)` (an `Array<S>` to host f64 data) and the residual bookkeeping of the loops (`records`,
// `read_history`, `rn_host`).
//
// Every function mirrors the CPU routine of the same name in `bowshock-core` expression by expression; the array code is
// the equation, the doc comment states it.

use bowshock_core::flux::{Limiter, LOW_MACH_FLOOR, SHOCK_SENSOR_HI, SHOCK_SENSOR_LO};
use bowshock_core::gas::GAMMA;
use bowshock_core::grid::Grid;
use bowshock_core::solver::{Bc, Block, Config};
use crate::Staging;
use luma::{concat, select, Array, Context, Fields, PadMode, Result};

/// Array of one field (`rho`, `u`, ...) over a block of cells or faces.
pub type A = Array<S>;
/// Four fields: primitive `[rho, u, v, p]`, conservative `[rho, rho u, rho v, E]` or a flux. A record of arrays
/// (`luma::Fields`): `slice`, `pad`, `assign` and `cache` apply to the four fields at once.
pub type Rec = [A; 4];
/// The loop state: the conserved record and the density residual of the step that produced it (one element).
pub type State = [A; 5];

fn rec(f: impl FnMut(usize) -> A) -> Rec {
    std::array::from_fn(f)
}

/// The conserved record of a loop state.
fn unpack(s: &State) -> Rec {
    rec(|k| s[k].clone())
}

fn pack(u: Rec, rn: A) -> State {
    let [u0, u1, u2, u3] = u;
    [u0, u1, u2, u3, rn]
}

/// The loop state of host data: the conserved state `u` (cell-major, `[rho, rho u, rho v, E]`) and a zero residual.
fn upload_state(ctx: &Context, u: &[[f64; 4]], shape: [usize; 2]) -> Result<State> {
    let comp = |k: usize| up(ctx, &u.iter().map(|q| q[k]).collect::<Vec<_>>(), shape);
    Ok([comp(0)?, comp(1)?, comp(2)?, comp(3)?, up(ctx, &[0.0], 1)?])
}

fn download_state(s: &State) -> Result<Vec<[f64; 4]>> {
    let v: Vec<Vec<f64>> = (0..4).map(|k| to_f64(&s[k])).collect::<Result<_>>()?;
    Ok((0..v[0].len()).map(|c| [v[0][c], v[1][c], v[2][c], v[3][c]]).collect())
}

const G1: f64 = GAMMA - 1.0;

// ---------------------------------------------------------------------------------------------------------------------
// gas.rs
// ---------------------------------------------------------------------------------------------------------------------

/// `rho = q0, u = q1 / rho, v = q2 / rho, p = (g - 1) (E - rho (u^2 + v^2) / 2)`.
pub fn cons_to_prim(q: Rec) -> Rec {
    let u = &q[1] / &q[0];
    let v = &q[2] / &q[0];
    let p = c(G1) * (&q[3] - c(0.5) * &q[0] * (&u * &u + &v * &v));
    let [rho, ..] = q;
    [rho, u, v, p]
}

/// `F.n = [rho u_n, rho u u_n + p n_x, rho v u_n + p n_r, (E + p) u_n]` with `u_n = u n_x + v n_r`.
pub fn physical_flux(w: &Rec, nx: &A, nr: &A) -> Rec {
    let [rho, u, v, p] = w;
    let un = u * nx + v * nr;
    let e = p / c(G1) + c(0.5) * rho * (u * u + v * v);
    flux_from(w, &un, &e, nx, nr)
}

/// [`physical_flux`] of a state whose normal velocity `u_n` and total energy `E` are known.
fn flux_from(w: &Rec, un: &A, e: &A, nx: &A, nr: &A) -> Rec {
    let [rho, u, v, p] = w;
    [rho * un, rho * u * un + p * nx, rho * v * un + p * nr, (e + p) * un]
}

/// One side of a face with the quantities both Riemann solvers need, built once: normal velocity `u_n = u n_x + v n_r`,
/// total energy `E = p / (g - 1) + rho (u^2 + v^2) / 2`, sound speed `c = sqrt(g p / rho)` and the flux `F.n`.
struct Side<'a> {
    w: &'a Rec,
    un: A,
    e: A,
    c: A,
    f: Rec,
}

impl<'a> Side<'a> {
    fn new(w: &'a Rec, nx: &A, nr: &A) -> Side<'a> {
        let [rho, u, v, p] = w;
        let un = u * nx + v * nr;
        let e = p / c(G1) + c(0.5) * rho * (u * u + v * v);
        let f = flux_from(w, &un, &e, nx, nr);
        Side { w, un, e, c: (c(GAMMA) * p / rho).sqrt(), f }
    }
}

/// `w` reflected about a wall of unit normal `(n_x, n_r)`: `v - 2 (v.n) n`.
fn reflect(w: &Rec, nx: &A, nr: &A) -> Rec {
    let un = &w[1] * nx + &w[2] * nr;
    [w[0].clone(), &w[1] - c(2.0) * &un * nx, &w[2] - c(2.0) * &un * nr, w[3].clone()]
}

// ---------------------------------------------------------------------------------------------------------------------
// flux.rs
// ---------------------------------------------------------------------------------------------------------------------

/// Limited slope `phi(a, b)`, `a = q_i - q_{i-1}`, `b = q_{i+1} - q_i`; zero unless `a b > 0`.
/// minmod: the smaller of `a`, `b` in modulus. van Albada: `(a (b^2 + e) + b (a^2 + e)) / (a^2 + b^2 + 2 e)`, `e = 1e-14`.
fn limited_slope(a: &A, b: &A, lim: Limiter) -> A {
    let same_sign = (a * b).gt(c(0.0));
    match lim {
        Limiter::FirstOrder => unreachable!("first order reconstruction has no slopes"),
        Limiter::Minmod => select(&same_sign, select(&a.abs().lt(&b.abs()), a, b), c(0.0)),
        Limiter::VanAlbada => {
            let eps = 1e-14;
            let slope = (a * (b * b + c(eps)) + b * (a * a + c(eps))) / (a * a + b * b + c(2.0 * eps));
            select(&same_sign, slope, c(0.0))
        }
    }
}

/// MUSCL reconstruction at the face between the cells `m1 | p0` (`m2`, `p1` the outer neighbours):
/// `l = m1 + phi(m1 - m2, p0 - m1) / 2`, `r = p0 - phi(p0 - m1, p1 - p0) / 2`;
/// a face whose reconstructed density or pressure is not positive falls back to the cell value.
fn reconstruct(m2: &Rec, m1: &Rec, p0: &Rec, p1: &Rec, lim: Limiter) -> (Rec, Rec) {
    if lim == Limiter::FirstOrder {
        return (m1.clone(), p0.clone());
    }
    let l = rec(|k| &m1[k] + c(0.5) * limited_slope(&(&m1[k] - &m2[k]), &(&p0[k] - &m1[k]), lim));
    let r = rec(|k| &p0[k] - c(0.5) * limited_slope(&(&p0[k] - &m1[k]), &(&p1[k] - &p0[k]), lim));
    let ok_l = l[0].gt(c(0.0)) * l[3].gt(c(0.0));
    let ok_r = r[0].gt(c(0.0)) * r[3].gt(c(0.0));
    (rec(|k| select(&ok_l, &l[k], &m1[k])), rec(|k| select(&ok_r, &r[k], &p0[k])))
}

/// HLLC flux for the 2-D Euler equations projected on the unit normal `(n_x, n_r)` (Toro ch. 10), wave speeds
/// `S_L = min(u_nL - c_L, u~_n - c~)`, `S_R = max(u_nR + c_R, u~_n + c~)` from Roe averages (`sqrt(rho)` weights):
/// `F = F_L` for `S_L >= 0`, `F_R` for `S_R <= 0`, else `F_K + S_K (U*_K - U_K)` of the side `K` of the contact speed `S_M`.
fn hllc_speeds(l: &Side, r: &Side, nx: &A, nr: &A) -> [A; 3] {
    let [rl, ul, vl, pl] = l.w;
    let [rr, ur, vr, pr] = r.w;
    let (unl, unr, cl, cr, el, er) = (&l.un, &r.un, &l.c, &r.c, &l.e, &r.e);

    // Roe averages
    let sql = rl.sqrt();
    let sqr = rr.sqrt();
    let inv = c(1.0) / (&sql + &sqr);
    let uh = (&sql * ul + &sqr * ur) * &inv;
    let vh = (&sql * vl + &sqr * vr) * &inv;
    let hl = (el + pl) / rl;
    let hr = (er + pr) / rr;
    let hh = (&sql * &hl + &sqr * &hr) * &inv;
    let ch2 = c(G1) * (&hh - c(0.5) * (&uh * &uh + &vh * &vh));
    let ch = ch2.maximum(c(1e-30)).sqrt();
    let unh = &uh * nx + &vh * nr;

    let sl = (unl - cl).minimum(&unh - &ch);
    let sr = (unr + cr).maximum(&unh + &ch);
    let sl_un = &sl - unl;
    let sr_un = &sr - unr;
    let sm = (pr - pl + rl * unl * &sl_un - rr * unr * &sr_un) / (rl * &sl_un - rr * &sr_un);
    [sl, sr, sm]
}

/// The HLLC flux from the wave speeds `[S_L, S_R, S_M]` of [`hllc_speeds`].
fn hllc_flux(l: &Side, r: &Side, nx: &A, nr: &A, speeds: &[A; 3]) -> Rec {
    let [sl, sr, sm] = speeds;
    let sl_un = sl - &l.un;
    let sr_un = sr - &r.un;

    // `F_K + S_K (U*_K - U_K)`, `U*_K = fac [1, u + (S_M - u_n) n_x, v + (S_M - u_n) n_r, E / rho + (S_M - u_n) (S_M + p / (rho (S_K - u_n)))]`
    let star = |k: &Side, s: &A, s_un: &A| -> Rec {
        let [r, u, v, p] = k.w;
        let (f, e) = (&k.f, &k.e);
        let fac = r * s_un / (s - sm);
        let sm_un = sm - &k.un;
        [
            &f[0] + s * (&fac - r),
            &f[1] + s * (&fac * (u + &sm_un * nx) - r * u),
            &f[2] + s * (&fac * (v + &sm_un * nr) - r * v),
            &f[3] + s * (&fac * (e / r + &sm_un * (sm + p / (r * s_un))) - e),
        ]
    };
    let star_l = star(l, sl, &sl_un);
    let star_r = star(r, sr, &sr_un);

    let left = sl.ge(c(0.0));
    let right = sr.le(c(0.0));
    let contact_left = sm.ge(c(0.0));
    rec(|k| select(&left, &l.f[k], select(&right, &r.f[k], select(&contact_left, &star_l[k], &star_r[k]))))
}

/// Rusanov flux `F = (F_L + F_R) / 2 - s_max (U_R - U_L) / 2`, `s_max = max(|u_nL| + c_L, |u_nR| + c_R)`.
fn rusanov(l: &Side, r: &Side) -> Rec {
    let [rl, ul, vl, _] = l.w;
    let [rr, ur, vr, _] = r.w;
    let smax = (l.un.abs() + &l.c).maximum(r.un.abs() + &r.c);
    let dq = [rr - rl, rr * ur - rl * ul, rr * vr - rl * vl, &r.e - &l.e];
    rec(|k| c(0.5) * (&l.f[k] + &r.f[k]) - c(0.5) * &smax * &dq[k])
}

/// Weight of the Rusanov flux: `s = |p_R - p_L| / min(p_L, p_R)`, `t = clamp((s - lo) / (hi - lo), 0, 1)`, `theta = t^2 (3 - 2 t)`.
fn shock_weight(pl: &A, pr: &A) -> A {
    let s = (pr - pl).abs() / pl.minimum(pr);
    let t = ((s - c(SHOCK_SENSOR_LO)) / c(SHOCK_SENSOR_HI - SHOCK_SENSOR_LO)).maximum(c(0.0)).minimum(c(1.0));
    &t * &t * (c(3.0) - c(2.0) * &t)
}

/// Low-Mach scaling of the face-normal velocity jump (Thornber et al. 2008): `z = clamp(max(M_L, M_R), 0.05, 1)`,
/// `u_n' = u_avg + z (u_n - u_avg)` on both sides, untouched for `z >= 1`.
fn low_mach_states(l: Rec, r: Rec, nx: &A, nr: &A) -> (Rec, Rec) {
    let unl = &l[1] * nx + &l[2] * nr;
    let unr = &r[1] * nx + &r[2] * nr;
    let ml = (&l[1] * &l[1] + &l[2] * &l[2]).sqrt() / (c(GAMMA) * &l[3] / &l[0]).sqrt();
    let mr = (&r[1] * &r[1] + &r[2] * &r[2]).sqrt() / (c(GAMMA) * &r[3] / &r[0]).sqrt();
    let z = ml.maximum(&mr).maximum(c(LOW_MACH_FLOOR)).minimum(c(1.0));
    let active = z.lt(c(1.0));
    let avg = c(0.5) * (&unl + &unr);
    let dl = &avg + &z * (&unl - &avg) - &unl;
    let dr = &avg + &z * (&unr - &avg) - &unr;
    let fix = |w: Rec, d: &A| -> Rec {
        let [rho, u, v, p] = w;
        [rho, select(&active, &u + d * nx, &u), select(&active, &v + d * nr, &v), p]
    };
    (fix(l, &dl), fix(r, &dr))
}

/// Complete face kernel: MUSCL + (low-Mach) + `(1 - theta) HLLC + theta Rusanov`.
///
/// `split = false` is one expression (what the equations say): Luma fuses it into one kernel of ~115 operations.
/// `split = true` evaluates the reconstructed states and the shock weight first (`cache`), the flux reads them from memory.
fn face_flux_lm(m2: &Rec, m1: &Rec, p0: &Rec, p1: &Rec, lim: Limiter, nx: &A, nr: &A, low_mach: bool, split: bool) -> Rec {
    let (l, r) = reconstruct(m2, m1, p0, p1, lim);
    let (l, r) = if low_mach { low_mach_states(l, r, nx, nr) } else { (l, r) };
    let t = shock_weight(&m1[3], &p0[3]);
    let (l, r, t) = if split { (l, r, t).cache() } else { (l, r, t) };
    let (sl, sr) = (Side::new(&l, nx, nr), Side::new(&r, nx, nr));
    let hllc = hllc_flux(&sl, &sr, nx, nr, &hllc_speeds(&sl, &sr, nx, nr));
    let rus = rusanov(&sl, &sr);
    rec(|k| (c(1.0) - &t) * &hllc[k] + &t * &rus[k])
}

// ---------------------------------------------------------------------------------------------------------------------
// bc.rs
// ---------------------------------------------------------------------------------------------------------------------

/// Characteristic far field. With the outward normal `n`, `u_n,i` and `c_i` of the interior state `i` and `e` the free stream:
/// `R_out = u_n,i + 2 c_i / (g - 1)`, `R_in = u_n,e - 2 c_e / (g - 1)`, `u_n,b = (R_out + R_in) / 2`,
/// `c_b = (g - 1) (R_out - R_in) / 4`; entropy and tangential velocity come from the interior for `u_n,b >= 0`, else from the
/// free stream (state `ref`): `rho_b = rho_ref (c_b / c_ref)^(2 / (g - 1))`, `p_b = rho_b c_b^2 / g`,
/// `v_b = v_ref + (u_n,b - u_n,ref) n`. `M_n = u_n,i / c_i <= -1`: free stream, `>= 1`: interior.
fn characteristic_ghost(fs: &[f64; 4], interior: &Rec, nx: &A, nr: &A) -> Rec {
    let ci = (c(GAMMA) * &interior[3] / &interior[0]).sqrt();
    let uni = &interior[1] * nx + &interior[2] * nr;
    let m = &uni / &ci;
    let ce = (GAMMA * fs[3] / fs[0]).sqrt();
    let une = c(fs[1]) * nx + c(fs[2]) * nr;
    let r_out = &uni + c(2.0) * &ci / c(G1);
    let r_in = &une - c(2.0 * ce / G1);
    let un_b = c(0.5) * (&r_out + &r_in);
    let c_b = (c(0.25 * G1) * (&r_out - &r_in)).maximum(c(1e-8));
    let from_interior = un_b.ge(c(0.0));
    let rf = [
        select(&from_interior, &interior[0], c(fs[0])),
        select(&from_interior, &interior[1], c(fs[1])),
        select(&from_interior, &interior[2], c(fs[2])),
    ];
    let un_ref = select(&from_interior, &uni, &une);
    let c_ref = select(&from_interior, &ci, c(ce));
    let q = &c_b / &c_ref;
    let q2 = &q * &q;
    let rho = &rf[0] * (&q2 * &q2 * &q);
    let dn = &un_b - &un_ref;
    let riemann = [rho.clone(), &rf[1] + &dn * nx, &rf[2] + &dn * nr, &rho * &c_b * &c_b / c(GAMMA)];
    let super_in = m.le(c(-1.0));
    let super_out = m.ge(c(1.0));
    rec(|k| select(&super_in, c(fs[k]), select(&super_out, &interior[k], &riemann[k])))
}

/// Fixed static-pressure outflow: `p = p_back` where `M_n < 1`, interior state for `M_n >= 1`.
fn pressure_outflow_ghost(p_back: f64, interior: &Rec, nx: &A, nr: &A) -> Rec {
    let ci = (c(GAMMA) * &interior[3] / &interior[0]).sqrt();
    let m = (&interior[1] * nx + &interior[2] * nr) / &ci;
    [interior[0].clone(), interior[1].clone(), interior[2].clone(), select(&m.ge(c(1.0)), &interior[3], c(p_back))]
}

// ---------------------------------------------------------------------------------------------------------------------
// solver.rs
// ---------------------------------------------------------------------------------------------------------------------

/// Geometry of one domain boundary: the two ghost layers (outermost first), unit normal `(n_x, n_r)` of the boundary face and
/// the outward normal; constant ghost states for `Bc::Freestream`.
struct Boundary {
    nx: A,
    nr: A,
    onx: A,
    onr: A,
    fixed: Option<Rec>,
}

/// Face geometry and cell metrics, uploaded once from the grid.
struct Geo {
    /// i-faces `[ni + 1, nj]`: normal, length, flux weight `len r_mid`.
    inx: A,
    inr: A,
    ilen: A,
    iw: A,
    /// j-faces `[ni, nj + 1]`.
    jnx: A,
    jnr: A,
    jlen: A,
    jw: A,
    /// `1 / V` and `A` per cell.
    inv_v: A,
    area: A,
    /// `[imin, imax, jmin, jmax]`
    bnd: [Boundary; 4],
}

/// The explicit solver: state, geometry and the loops that advance it, with an optional solid block (base / wake grids).
/// Mirrors `bowshock_core::solver::Solver` with `Scheme::Explicit` (`eval_residual` and `rk3_step`).
pub struct LazySolver {
    ctx: Context,
    pub ni: usize,
    pub nj: usize,
    cfg: Config,
    pub low_mach: bool,
    /// How the evaluation is ordered (see [`Staging`]).
    pub staging: Staging,
    geo: Geo,
    state: Option<State>,
    pub iter: usize,
    block: Option<Block>,
    /// Number of fluid cells (the residual norm divides by it).
    n_active: usize,
}

impl LazySolver {
    /// Uploads the geometry (rounded from the grid's f64 metrics to `S`) and starts from the free stream.
    pub fn new(ctx: &Context, grid: &Grid, cfg: &Config) -> Result<LazySolver> {
        let (ni, nj) = (grid.ni, grid.nj);
        let face = |f: &[bowshock_core::grid::Face], shape: [usize; 2], g: &dyn Fn(&bowshock_core::grid::Face) -> f64| {
            let v: Vec<f64> = f.iter().map(g).collect();
            up(ctx, &v, shape)
        };
        let boundary = |side: usize| -> Result<Boundary> {
            // faces of the boundary, and the shape of the two ghost layers
            let (faces, shape, outward): (Vec<bowshock_core::grid::Face>, [usize; 2], f64) = match side {
                0 => ((0..nj).map(|j| grid.iface[j * (ni + 1)]).collect(), [2, nj], -1.0),
                1 => ((0..nj).map(|j| grid.iface[j * (ni + 1) + ni]).collect(), [2, nj], 1.0),
                2 => ((0..ni).map(|i| grid.jface[i]).collect(), [ni, 2], -1.0),
                _ => ((0..ni).map(|i| grid.jface[nj * ni + i]).collect(), [ni, 2], 1.0),
            };
            // the layers are laid out `[layer, cell]` for i boundaries and `[cell, layer]` for j boundaries
            let layered = |g: &dyn Fn(&bowshock_core::grid::Face) -> f64| -> Result<A> {
                let v: Vec<f64> = if side < 2 { faces.iter().flat_map(|f| [g(f), g(f)]).collect() } else { [0, 1].iter().flat_map(|_| faces.iter().map(g)).collect() };
                up(ctx, &v, shape)
            };
            let fixed = if cfg.bc[side] == Bc::Freestream {
                let fs = cfg.freestream;
                let mut v = Vec::new();
                for k in 0..4 {
                    v.push(up(ctx, &vec![fs[k]; shape[0] * shape[1]], shape)?);
                }
                let mut it = v.into_iter();
                Some(rec(|_| it.next().unwrap()))
            } else {
                None
            };
            Ok(Boundary { nx: layered(&|f| f.nx)?, nr: layered(&|f| f.nr)?, onx: layered(&|f| outward * f.nx)?, onr: layered(&|f| outward * f.nr)?, fixed })
        };
        let geo = Geo {
            inx: face(&grid.iface, [ni + 1, nj], &|f| f.nx)?,
            inr: face(&grid.iface, [ni + 1, nj], &|f| f.nr)?,
            ilen: face(&grid.iface, [ni + 1, nj], &|f| f.len)?,
            iw: face(&grid.iface, [ni + 1, nj], &|f| f.w())?,
            jnx: face(&grid.jface, [ni, nj + 1], &|f| f.nx)?,
            jnr: face(&grid.jface, [ni, nj + 1], &|f| f.nr)?,
            jlen: face(&grid.jface, [ni, nj + 1], &|f| f.len)?,
            jw: face(&grid.jface, [ni, nj + 1], &|f| f.w())?,
            inv_v: up(ctx, &grid.vol.iter().map(|v| 1.0 / v).collect::<Vec<_>>(), [ni, nj])?,
            area: up(ctx, &grid.area, [ni, nj])?,
            bnd: [boundary(0)?, boundary(1)?, boundary(2)?, boundary(3)?],
        };
        let mut s = LazySolver { ctx: ctx.clone(), ni, nj, cfg: cfg.clone(), low_mach: false, staging: DEFAULT_STAGING, geo, state: None, iter: 0, block: None, n_active: ni * nj };
        let q = bowshock_core::gas::prim_to_cons(&cfg.freestream);
        s.set_state(&vec![q; ni * nj])?;
        Ok(s)
    }

    /// A lazy solver in the state of a host solver: grid, config, low-Mach flag, block, conserved state and iteration count.
    pub fn from_host(ctx: &Context, s: &bowshock_core::solver::Solver) -> Result<LazySolver> {
        let mut l = LazySolver::new(ctx, &s.grid, &s.cfg)?;
        l.set_low_mach(s.low_mach);
        l.set_block(s.block())?;
        l.set_state(&s.u)?;
        l.iter = s.iter;
        Ok(l)
    }

    /// Declares the solid block (`None`: no block). Same range checks as `Solver::set_block`. The block's cells stay at their
    /// state (residual and time step zero) and are left out of the residual norm.
    pub fn set_block(&mut self, block: Option<Block>) -> Result<()> {
        if let Some(b) = block {
            assert!(
                b.i_end >= 1
                    && b.i_end + 2 <= self.ni
                    && b.j_end >= 2
                    && b.j_end + 2 <= self.nj
                    && b.j_end > b.j_sting
                    && (b.j_sting == 0 || b.j_sting >= 2),
                "block out of range"
            );
        }
        self.block = block;
        self.n_active = match block {
            Some(b) => self.ni * self.nj - b.i_end * b.j_end - (self.ni - b.i_end) * b.j_sting,
            None => self.ni * self.nj,
        };
        Ok(())
    }

    /// `sqrt(sum (d rho / dt)^2 / N)`, `N` the number of fluid cells.
    fn residual_norm(&self, r0: &A) -> A {
        ((r0 * r0).sum() / c(self.n_active as f64)).sqrt()
    }

    /// Zeros of `shape` over the region of every field of `r`.
    fn zero_region(&self, r: &Rec, shape: [usize; 2], region: impl Fn(&A) -> A) {
        if shape[0] == 0 || shape[1] == 0 {
            return;
        }
        for k in 0..4 {
            region(&r[k]).assign(c(0.0)).expect("zero region");
        }
    }

    /// Solid cells of the two layers below a body or sting surface take the reflection of the fluid cells above it about the
    /// j-face of the surface (`fill_block_ghosts`): the layers `col_start(i) - 2`, `col_start(i) - 1` get the cells
    /// `col_start(i) + 1`, `col_start(i)`. Written after the domain ghosts, as on the CPU.
    fn assign_block_ghosts(&self, b: Block, p: &Rec, w: &Rec) {
        let g = &self.geo;
        for (i0, i1, j0) in [(0, b.i_end, b.j_end), (b.i_end, self.ni, b.j_sting)] {
            if j0 == 0 {
                continue;
            }
            let dup = |a: A| concat(1, [&a, &a]);
            let nx = dup(g.jnx.slice(0, i0..i1).slice(1, j0..j0 + 1));
            let nr = dup(g.jnr.slice(0, i0..i1).slice(1, j0..j0 + 1));
            let src = rec(|k| p[k].slice(0, i0..i1).slice(1, j0..).pad([(0, 0), (2, 0)], PadMode::Mirror).slice(1, 0..2));
            let region = w.slice(0, i0 + 2..i1 + 2).slice(1, j0..j0 + 2);
            region.assign(reflect(&src, &nx, &nr)).expect("block ghost assign");
        }
    }

    /// The face fluxes next to the solid block, written into the i / j flux records `fi`, `fj`: the solid faces are zero, and
    /// the base-wall faces `i_end`, `i_end + 1` (rows `j_sting..j_end`) see the solid stencil cells as the reflection of the
    /// fluid cells about the base face:
    ///
    /// ```text
    /// face i_end:     (m2, m1, p0, p1) = (R c1, R c0, c0, c1)
    /// face i_end + 1: (m2, m1, p0, p1) = (R c0,  c0, c1, c2)         c_k the cell i_end + k, R the reflection
    /// ```
    fn block_fluxes(&self, b: Block, w: &Rec, fi: &Rec, fj: &Rec, lim: Limiter) {
        let ni = self.ni;
        let g = &self.geo;
        let nb = b.j_end - b.j_sting;
        // base wall faces: both faces use the normal of the base face for the reflection, their own for the flux
        let rows = |a: &A, i0: usize, i1: usize| a.slice(0, i0..i1).slice(1, b.j_sting..b.j_end);
        let (bx, br) = (rows(&g.inx, b.i_end, b.i_end + 1), rows(&g.inr, b.i_end, b.i_end + 1));
        let (fx, fr) = (rows(&g.inx, b.i_end, b.i_end + 2), rows(&g.inr, b.i_end, b.i_end + 2));
        let cells = |k: usize| rec(|f| w[f].slice(0, b.i_end + 2 + k..b.i_end + 3 + k).slice(1, b.j_sting + 2..b.j_end + 2));
        let (c0, c1, c2) = (cells(0), cells(1), cells(2));
        let join = |a: &Rec, bb: &Rec| rec(|f| concat(0, [&a[f], &bb[f]]));
        let (rc0, rc1) = (reflect(&c0, &bx, &br), reflect(&c1, &bx, &br));
        let (m2, m1) = (join(&rc1, &rc0), join(&rc0, &c0));
        let (p0, p1) = (join(&c0, &c1), join(&c1, &c2));
        let base = face_flux_lm(&m2, &m1, &p0, &p1, lim, &fx, &fr, self.low_mach, self.staging == Staging::Split);
        // solid faces
        self.zero_region(fi, [ni + 1, b.j_sting], |a| a.slice(1, 0..b.j_sting));
        self.zero_region(fi, [b.i_end, nb], |a| a.slice(0, 0..b.i_end).slice(1, b.j_sting..b.j_end));
        for k in 0..4 {
            fi[k].slice(0, b.i_end..b.i_end + 2).slice(1, b.j_sting..b.j_end).assign(&base[k]).expect("base wall flux assign");
        }
        self.zero_region(fj, [b.i_end, b.j_end], |a| a.slice(0, 0..b.i_end).slice(1, 0..b.j_end));
        self.zero_region(fj, [ni - b.i_end, b.j_sting], |a| a.slice(0, b.i_end..ni).slice(1, 0..b.j_sting));
    }

    pub fn set_low_mach(&mut self, on: bool) {
        self.low_mach = on;
    }

    /// Overwrites the conserved state and restarts the iteration counter.
    pub fn set_state(&mut self, u: &[[f64; 4]]) -> Result<()> {
        self.state = Some(upload_state(&self.ctx, u, [self.ni, self.nj])?);
        self.iter = 0;
        Ok(())
    }

    /// The conserved state on the host.
    pub fn state_f64(&self) -> Result<Vec<[f64; 4]>> {
        download_state(self.state.as_ref().expect("state"))
    }

    /// Writes the two ghost layers of boundary `side` (`[imin, imax, jmin, jmax]`) into the padded primitives `w` with a
    /// region assign. `p` is the unpadded primitive state: the first interior cell `edge` and the cell `mirror` that a
    /// wall reflects (`pad` with `Edge` / `Mirror` read both). `Bc::Extrapolate` keeps the `Edge` padding of `w`. Layers
    /// are outermost first, as the cells they stand for. `w` is a lazy expression, so nothing is launched: `w` becomes the
    /// padded state with the ghost expressions in place, and fuses into whatever reads it.
    fn assign_ghosts(&self, side: usize, p: &Rec, w: &Rec) {
        let (ni, nj) = (self.ni, self.nj);
        let view = |a: &A, mode: PadMode| -> A {
            match side {
                0 => a.pad([(2, 0)], mode).slice(0, 0..2),
                1 => a.pad([(0, 2)], mode).slice(0, ni..ni + 2),
                2 => a.pad([(0, 0), (2, 0)], mode).slice(1, 0..2),
                _ => a.pad([(0, 0), (0, 2)], mode).slice(1, nj..nj + 2),
            }
        };
        let region = match side {
            0 => w.slice(0, ..2).slice(1, 2..nj + 2),
            1 => w.slice(0, ni + 2..).slice(1, 2..nj + 2),
            2 => w.slice(1, ..2).slice(0, 2..ni + 2),
            _ => w.slice(1, nj + 2..).slice(0, 2..ni + 2),
        };
        let b = &self.geo.bnd[side];
        let fs = self.cfg.freestream;
        let ghost = match self.cfg.bc[side] {
            Bc::Freestream => b.fixed.clone().expect("fixed ghost"),
            Bc::Extrapolate => return,
            Bc::SlipWall => reflect(&rec(|k| view(&p[k], PadMode::Mirror)), &b.nx, &b.nr),
            Bc::Characteristic => characteristic_ghost(&fs, &rec(|k| view(&p[k], PadMode::Edge)), &b.onx, &b.onr),
            Bc::PressureOutflow => pressure_outflow_ghost(fs[3], &rec(|k| view(&p[k], PadMode::Edge)), &b.onx, &b.onr),
        };
        region.assign(ghost).expect("ghost layer assign");
    }

    /// `res = dU/dt` of the conserved state `u` (`[ni, nj]` fields) and, if asked, the local time step.
    ///
    /// ```text
    /// dU/dt = -(1/V) sum_faces (len r_mid) F.n  +  (0, 0, p A / V, 0)        dt = cfl A / (1/2 sum_faces (|u.n| + c) len)
    /// ```
    pub fn eval_residual(&self, u: &Rec, lim: Limiter, compute_dt: bool) -> (Rec, Option<A>) {
        let (p, w) = self.padded(u);
        self.residual_from(&p, &w, lim, compute_dt)
    }

    /// Steps 1 of `eval_residual`: the primitives `p = cons_to_prim(u)` and the padded primitives `w` with the domain ghosts,
    /// then the block ghosts, in place (the `Staging` cache applied).
    fn padded(&self, u: &Rec) -> (Rec, Rec) {
        // 1. primitives, padded by two cells on every side; the ghost layers of each boundary are one region assign
        let p = cons_to_prim(u.clone());
        let w = p.pad([(2, 2), (2, 2)], PadMode::Edge);
        for side in 0..4 {
            self.assign_ghosts(side, &p, &w);
        }
        if let Some(b) = self.block {
            self.assign_block_ghosts(b, &p, &w);
        }
        // `Lines` and up: evaluate the padded record once. Without it every reader recomputes the primitives and all four ghost
        // expressions at each stencil cell (200x100 f32: 67-91 launches per RK3 step without it, 26-30 with it; README "Speed").
        let w = if self.staging == Staging::Fusion { w } else { w.cache() };
        (p, w)
    }

    /// Steps 2 to 5 of `eval_residual`: face fluxes, cell residual and time step from the primitives `p` and the padded `w`.
    fn residual_from(&self, p: &Rec, w: &Rec, lim: Limiter, compute_dt: bool) -> (Rec, Option<A>) {
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.geo;

        // 2. the cells along each grid line with their ghosts: i lines `[ni + 4, nj]`, j lines `[ni, nj + 4]`
        let wi = w.slice(1, 2..nj + 2);
        let wj = w.slice(0, 2..ni + 2);

        // 3. face fluxes: face `f` sees the cells `f - 2 .. f + 1` of its line
        let split = self.staging == Staging::Split;
        let fi = face_flux_lm(&wi.slice(0, 0..ni + 1), &wi.slice(0, 1..ni + 2), &wi.slice(0, 2..ni + 3), &wi.slice(0, 3..ni + 4), lim, &g.inx, &g.inr, self.low_mach, split);
        let fj = face_flux_lm(&wj.slice(1, 0..nj + 1), &wj.slice(1, 1..nj + 2), &wj.slice(1, 2..nj + 3), &wj.slice(1, 3..nj + 4), lim, &g.jnx, &g.jnr, self.low_mach, split);
        if let Some(b) = self.block {
            self.block_fluxes(b, w, &fi, &fj, lim);
        }
        // `Fluxes` and up: evaluate the face fluxes. The residual and the stage update then fuse into the next stage's padded
        // record: three kernels per stage, 14 launches per step instead of 26-30 (the flux kernel read through four stencil
        // views is cut at the 1024-operation fusion limit otherwise; README "Speed").
        let (fi, fj) = if matches!(self.staging, Staging::Fluxes | Staging::Split) { (fi.cache(), fj.cache()) } else { (fi, fj) };

        // 4. cell residual
        let (wr, wl) = (g.iw.slice(0, 1..ni + 1), g.iw.slice(0, 0..ni));
        let (wt, wb) = (g.jw.slice(1, 1..nj + 1), g.jw.slice(1, 0..nj));
        let mut res = rec(|k| {
            let (gr, gl) = (fi[k].slice(0, 1..ni + 1), fi[k].slice(0, 0..ni));
            let (gt, gb) = (fj[k].slice(1, 1..nj + 1), fj[k].slice(1, 0..nj));
            -(&gr * &wr - &gl * &wl + &gt * &wt - &gb * &wb) * &g.inv_v
        });
        res[2] = &res[2] + &p[3] * &g.area * &g.inv_v; // axisymmetric pressure source p A / V

        // 5. local time step
        let dt = compute_dt.then(|| {
            let cs = (c(GAMMA) * &p[3] / &p[0]).sqrt();
            let lam = |nx: &A, nr: &A, len: &A| ((&p[1] * nx + &p[2] * nr).abs() + &cs) * len;
            let (fl_nx, fl_nr, fl_len) = (g.inx.slice(0, 0..ni), g.inr.slice(0, 0..ni), g.ilen.slice(0, 0..ni));
            let (fr_nx, fr_nr, fr_len) = (g.inx.slice(0, 1..ni + 1), g.inr.slice(0, 1..ni + 1), g.ilen.slice(0, 1..ni + 1));
            let (fb_nx, fb_nr, fb_len) = (g.jnx.slice(1, 0..nj), g.jnr.slice(1, 0..nj), g.jlen.slice(1, 0..nj));
            let (ft_nx, ft_nr, ft_len) = (g.jnx.slice(1, 1..nj + 1), g.jnr.slice(1, 1..nj + 1), g.jlen.slice(1, 1..nj + 1));
            let sum = lam(&fl_nx, &fl_nr, &fl_len) + lam(&fr_nx, &fr_nr, &fr_len) + lam(&fb_nx, &fb_nr, &fb_len) + lam(&ft_nx, &ft_nr, &ft_len);
            c(self.cfg.cfl) * &g.area / (c(0.5) * &sum)
        });
        // solid cells: residual and time step zero (region assigns, so nothing computed inside the solid reaches the state)
        if let Some(b) = self.block {
            self.zero_region(&res, [b.i_end, b.j_end], |a| a.slice(0, 0..b.i_end).slice(1, 0..b.j_end));
            self.zero_region(&res, [ni - b.i_end, b.j_sting], |a| a.slice(0, b.i_end..ni).slice(1, 0..b.j_sting));
            if let Some(dt) = &dt {
                if b.j_sting > 0 {
                    dt.slice(0, b.i_end..ni).slice(1, 0..b.j_sting).assign(c(0.0)).expect("dt zero");
                }
                dt.slice(0, 0..b.i_end).slice(1, 0..b.j_end).assign(c(0.0)).expect("dt zero");
            }
        }
        (res, dt)
    }

    /// One SSP-RK3 step with local time stepping; returns the new state and the density residual of the first stage
    /// `sqrt(sum (d rho / dt)^2 / N)`.
    ///
    /// ```text
    /// u1 = u0 + dt L(u0),   u2 = 3/4 u0 + 1/4 (u1 + dt L(u1)),   u3 = 1/3 u0 + 2/3 (u2 + dt L(u2))
    /// ```
    pub fn rk3(&self, u0: &Rec, lim: Limiter) -> (Rec, A) {
        let (r0, dt) = self.eval_residual(u0, lim, true);
        let dt = dt.expect("dt");
        let rn = self.residual_norm(&r0[0]);
        // stage 1: u1 = u0 + dt L(u0)
        let u1 = rec(|k| &u0[k] + &dt * &r0[k]);
        let (r1, _) = self.eval_residual(&u1, lim, false);
        // stage 2: u2 = 3/4 u0 + 1/4 (u1 + dt L(u1))
        let u2 = rec(|k| c(0.25) * (&u1[k] + &dt * &r1[k]) + c(0.75) * &u0[k]);
        let (r2, _) = self.eval_residual(&u2, lim, false);
        // stage 3: u3 = 1/3 u0 + 2/3 (u2 + dt L(u2))
        let u3 = rec(|k| c(2.0 / 3.0) * (&u2[k] + &dt * &r2[k]) + c(1.0 / 3.0) * &u0[k]);
        (u3, rn)
    }

    fn limiter(&self, iter: usize) -> Limiter {
        if iter < self.cfg.first_order_iters {
            Limiter::FirstOrder
        } else {
            self.cfg.limiter
        }
    }

    /// Number of iterations from `self.iter` that use the same limiter (the traced step is specific to the limiter).
    fn phase_len(&self, left: usize) -> usize {
        if self.iter < self.cfg.first_order_iters {
            left.min(self.cfg.first_order_iters - self.iter)
        } else {
            left
        }
    }

    /// `n` iterations as one `iterate_until_all` loop per limiter phase (the step is traced once and its kernels replayed,
    /// the density residual recorded on the GPU every iteration, the host waits once per `sync` iterations); returns the
    /// residual history.
    pub fn run(&mut self, n: usize, sync: usize) -> Result<Vec<f64>> {
        let mut history = Vec::with_capacity(n);
        let mut left = n;
        while left > 0 {
            let seg = self.phase_len(left);
            let lim = self.limiter(self.iter);
            let init = self.state.take().expect("state");
            let run = luma::iterate_until_all(init, records(luma::Until::new(seg).check_every(sync)), |s, _| {
                let (u, rn) = self.rk3(&unpack(s), lim);
                pack(u, rn)
            })?;
            history.extend(read_history(&run));
            self.state = Some(run.state);
            self.iter += seg;
            left -= seg;
        }
        Ok(history)
    }

    /// `n` iterations as one `iterate_all` replay per limiter phase (no residual history).
    pub fn run_replay(&mut self, n: usize) -> Result<()> {
        let mut left = n;
        while left > 0 {
            let seg = self.phase_len(left);
            let lim = self.limiter(self.iter);
            let init = self.state.take().expect("state");
            self.state = Some(luma::iterate_all(init.each_ref(), seg, |s| {
                let (u, rn) = self.rk3(&unpack(s), lim);
                pack(u, rn)
            }));
            self.iter += seg;
            left -= seg;
        }
        Ok(())
    }

    /// `n` iterations calling `rk3` every iteration and evaluating the new state with `eval_all` (no replay). Returns the
    /// evaluated residual scalars (read them with [`Residuals::read`]).
    pub fn run_eval(&mut self, n: usize) -> Result<Residuals> {
        let mut history = Vec::with_capacity(n);
        let mut s = self.state.take().expect("state");
        for _ in 0..n {
            let (u, rn) = self.rk3(&unpack(&s), self.limiter(self.iter));
            s = pack(u, rn);
            luma::eval_all(&s.iter().collect::<Vec<_>>())?;
            history.push(s[4].clone());
            self.iter += 1;
        }
        self.state = Some(s);
        Ok(Residuals(history))
    }

    /// Waits for the GPU.
    pub fn sync(&self) -> Result<()> {
        self.ctx.sync()
    }

    /// The residual `dU/dt` and time step of the current state, on the host (`[res; 4]` of `ni * nj`, `dt`).
    pub fn residual_f64(&self, lim: Limiter) -> Result<(Vec<[f64; 4]>, Vec<f64>)> {
        let u = unpack(self.state.as_ref().expect("state"));
        let (r, dt) = self.eval_residual(&u, lim, true);
        let r: Vec<Vec<f64>> = r.iter().map(to_f64).collect::<Result<_>>()?;
        let dt = to_f64(&dt.unwrap())?;
        Ok(((0..self.ni * self.nj).map(|c| [r[0][c], r[1][c], r[2][c], r[3][c]]).collect(), dt))
    }

    /// GPU time per kernel of one residual evaluation (profile) and where fusion cut the expressions.
    pub fn profile_residual(&self) -> Result<luma::Profile> {
        let u = unpack(self.state.as_ref().expect("state"));
        let lim = self.cfg.limiter;
        // warm up (compile) first, so that the profile shows launches only
        let _ = self.eval_residual(&u, lim, true);
        self.ctx.sync()?;
        let (out, profile) = self.ctx.profile(|| -> Result<()> {
            let (r, dt) = self.eval_residual(&u, lim, true);
            luma::eval_all(&r.iter().chain(dt.iter()).collect::<Vec<_>>())?;
            self.ctx.sync()
        })?;
        out?;
        Ok(profile)
    }
}

/// The density residual scalars of an `eval` loop, one per iteration, as evaluated GPU arrays.
pub struct Residuals(Vec<A>);

impl Residuals {
    /// Reads the residual history (waits for the GPU).
    pub fn read(&self) -> Result<Vec<f64>> {
        self.0.iter().map(|a| Ok(to_f64(a)?[0])).collect()
    }
}

impl LazySolver {
    /// GPU time per kernel of one full SSP-RK3 step (three residual evaluations and the stage updates) as the `eval` loop
    /// launches it, and where fusion cut the expressions.
    pub fn profile_step(&mut self) -> Result<luma::Profile> {
        self.run_eval(1)?;
        self.ctx.sync()?;
        let ctx = self.ctx.clone();
        let (out, profile) = ctx.profile(|| -> Result<()> {
            self.run_eval(1)?;
            self.ctx.sync()
        })?;
        out?;
        Ok(profile)
    }

    /// GPU time per kernel of `n` iterations of the `iterate_all` replay (the profile adds up all `n`), after a warm-up call
    /// that traces and compiles it.
    pub fn profile_replay(&mut self, n: usize) -> Result<luma::Profile> {
        self.run_replay(n)?;
        self.ctx.sync()?;
        let ctx = self.ctx.clone();
        let (out, profile) = ctx.profile(|| -> Result<()> {
            self.run_replay(n)?;
            self.ctx.sync()
        })?;
        out?;
        Ok(profile)
    }

    /// Host seconds to build the lazy expression of one SSP-RK3 step from the current state, `n` times, without evaluating
    /// anything (the part of an `eval` loop iteration that is Rust code building nodes).
    pub fn trace_seconds(&self, n: usize) -> f64 {
        let u = unpack(self.state.as_ref().expect("state"));
        let lim = self.cfg.limiter;
        let t0 = std::time::Instant::now();
        for _ in 0..n {
            std::hint::black_box(self.rk3(&u, lim));
        }
        t0.elapsed().as_secs_f64()
    }
}
