// The 3D explicit solver in array ops. This file is `include!`d twice by `gpu/mod.rs`, once per precision, into the same module as
// `body.rs` (it reuses `S`, `c`, `up`, `to_f64`, `A`, `limited_slope`, `shock_weight`, `G1`, `no_state`, `Residuals` from there and `records3` /
// `read_history3` of the precision module), so every name of this file carries a `3`.
//
// Every function mirrors the CPU routine of the same name in `solver3.rs`, `flux3.rs` and `bc3.rs`, expression by expression. The grid is
// `[ni, nj, nk]` arrays (axis 0 = `i`, the fastest; the host order of `Grid3::cell`), the padded primitive state `[ni + 4, nj + 4, nk + 4]`
// with two ghost layers per axis (`pidx3`). `Scheme3::Explicit` (SSP-RK3) is here, `Scheme3::Implicit` in `implicit3.rs`.

use crate::flux3::face_rot;
use crate::gas3::{prim_to_cons3, Cons3};
use crate::grid3::{Face3, Grid3};
use crate::solver3::{Config3, Scheme3, Solver3};

/// Five fields: primitive `[rho, u, v, w, p]`, conservative `[rho, rho u, rho v, rho w, E]` or a flux (a record of arrays like [`Rec`]).
pub type Rec3 = [A; 5];
/// Three components of a vector field (a face normal).
pub type Vec3 = [A; 3];
/// The loop state: the conserved record, the density residual of the step that produced it (one element), the CFL of the next iteration
/// (one element; the ramp of the implicit scheme is loop state, the explicit scheme passes it through), the SST scalars `k`, `omega` and the eddy
/// viscosity `mu_t` (cell arrays of an SST run, one-element placeholders otherwise) and the force monitor `[Fx, Fz, Mz]` of the state the step started from (three
/// elements, zero without a monitor: see [`LazySolver3::set_force_monitor`]).
pub type State3 = [A; 11];

fn rec3(f: impl FnMut(usize) -> A) -> Rec3 {
    std::array::from_fn(f)
}

fn unpack3(s: &State3) -> Rec3 {
    rec3(|k| s[k].clone())
}

fn pack3(u: Rec3, rn: A, cfl: A, turb: [A; 3], mon: A) -> State3 {
    let [u0, u1, u2, u3, u4] = u;
    let [k, om, mt] = turb;
    [u0, u1, u2, u3, u4, rn, cfl, k, om, mt, mon]
}

/// The turbulence part passed on unchanged (a placeholder is a fresh expression so the loop state does not alias).
fn turb_pass(s: &State3) -> [A; 3] {
    [&s[7] * c(1.0), &s[8] * c(1.0), &s[9] * c(1.0)]
}

/// The loop state of host data: the conserved state `u` (cell-major, `Grid3::cell` order), a zero residual and the CFL `cfl`.
fn upload_state3(ctx: &Context, u: &[Cons3], shape: [usize; 3], cfl: f64) -> Result<State3> {
    let comp = |k: usize| up(ctx, &u.iter().map(|q| q[k]).collect::<Vec<_>>(), shape);
    let zero = || up(ctx, &[0.0], 1);
    Ok([comp(0)?, comp(1)?, comp(2)?, comp(3)?, comp(4)?, zero()?, up(ctx, &[cfl], 1)?, zero()?, zero()?, zero()?, up(ctx, &[0.0, 0.0, 0.0], 3)?])
}

fn download_state3(s: &State3) -> Result<Vec<Cons3>> {
    let v: Vec<Vec<f64>> = (0..5).map(|k| to_f64(&s[k])).collect::<Result<_>>()?;
    Ok((0..v[0].len()).map(|q| [v[0][q], v[1][q], v[2][q], v[3][q], v[4][q]]).collect())
}

/// `select(cond, a, b)` of two records.
fn select3(cond: &Array<bool>, a: &Rec3, b: &Rec3) -> Rec3 {
    rec3(|k| select(cond, &a[k], &b[k]))
}

// ---------------------------------------------------------------------------------------------------------------------
// gas3.rs
// ---------------------------------------------------------------------------------------------------------------------

/// `rho = q0, u = q1 / rho, v = q2 / rho, w = q3 / rho, p = (g - 1) (E - rho (u^2 + v^2 + w^2) / 2)`. One division (`1 / rho`) and three products: a
/// double-float division is the most expensive operation of the record, and this one is evaluated for every cell of the padded state.
pub fn cons_to_prim3(q: Rec3) -> Rec3 {
    let inv = c(1.0) / &q[0];
    let u = &q[1] * &inv;
    let v = &q[2] * &inv;
    let w = &q[3] * &inv;
    let p = c(G1) * (&q[4] - c(0.5) * &q[0] * (&u * &u + &v * &v + &w * &w));
    let [rho, ..] = q;
    [rho, u, v, w, p]
}

/// One side of a face with the quantities both Riemann solvers need, built once: normal velocity `u_n = u.n`, total energy
/// `E = p / (g - 1) + rho |u|^2 / 2`, sound speed `c = sqrt(g p / rho)` and the flux `F.n = [rho u_n, rho u u_n + p n, (E + p) u_n]`.
struct Side3<'a> {
    w: &'a Rec3,
    un: A,
    e: A,
    c: A,
    f: Rec3,
}

impl<'a> Side3<'a> {
    fn new(w: &'a Rec3, n: &Vec3) -> Side3<'a> {
        let [rho, u, v, ww, p] = w;
        let un = u * &n[0] + v * &n[1] + ww * &n[2];
        let e = p / c(G1) + c(0.5) * rho * (u * u + v * v + ww * ww);
        let f = [rho * &un, rho * u * &un + p * &n[0], rho * v * &un + p * &n[1], rho * ww * &un + p * &n[2], (&e + p) * &un];
        Side3 { w, un, e, c: (c(GAMMA) * p / rho).sqrt(), f }
    }
}

/// `w` reflected about a wall of unit normal `n`: `v - 2 (v.n) n`.
fn reflect3(w: &Rec3, n: &Vec3) -> Rec3 {
    let un = &w[1] * &n[0] + &w[2] * &n[1] + &w[3] * &n[2];
    [w[0].clone(), &w[1] - c(2.0) * &un * &n[0], &w[2] - c(2.0) * &un * &n[1], &w[3] - c(2.0) * &un * &n[2], w[4].clone()]
}

// ---------------------------------------------------------------------------------------------------------------------
// flux3.rs
// ---------------------------------------------------------------------------------------------------------------------

/// MUSCL reconstruction at the face between the cells `m1 | p0` (`m2`, `p1` the outer neighbours). The lateral velocity `(v, w)` is limited in
/// the cylindrical frame of the face (`rot = [sin, cos]` of its azimuth: radial `v s + w co`, tangential `v co - w s`), the reconstructed states
/// are rotated back; a side whose reconstructed density or pressure is not positive, or whose slopes all vanish, is the cell value itself.
fn reconstruct3(m2: &Rec3, m1: &Rec3, p0: &Rec3, p1: &Rec3, lim: Limiter, rot: &[A; 2]) -> (Rec3, Rec3) {
    if lim == Limiter::FirstOrder {
        return (m1.clone(), p0.clone());
    }
    let (s, co) = (&rot[0], &rot[1]);
    // `(v, w) -> (v s + w co, v co - w s)` is its own inverse (a reflection for `s^2 + co^2 = 1`): the same map takes the states into the frame and back
    let turn = |w: &Rec3| -> Rec3 { [w[0].clone(), w[1].clone(), &w[2] * s + &w[3] * co, &w[2] * co - &w[3] * s, w[4].clone()] };
    let (a, b, cc, d) = (turn(m2), turn(m1), turn(p0), turn(p1));
    let l = rec3(|k| &b[k] + c(0.5) * limited_slope(&(&b[k] - &a[k]), &(&cc[k] - &b[k]), lim));
    let r = rec3(|k| &cc[k] - c(0.5) * limited_slope(&(&cc[k] - &b[k]), &(&d[k] - &cc[k]), lim));
    let ok_l = l[0].gt(c(0.0)) & l[4].gt(c(0.0));
    let ok_r = r[0].gt(c(0.0)) & r[4].gt(c(0.0));
    let same = |x: &Rec3, y: &Rec3| x[0].eq(&y[0]) & x[1].eq(&y[1]) & x[2].eq(&y[2]) & x[3].eq(&y[3]) & x[4].eq(&y[4]);
    let (same_l, same_r) = (same(&l, &b), same(&r, &cc));
    let (bl, br) = (turn(&l), turn(&r));
    (rec3(|k| select(&ok_l, select(&same_l, &m1[k], &bl[k]), &m1[k])), rec3(|k| select(&ok_r, select(&same_r, &p0[k], &br[k]), &p0[k])))
}

/// HLLC wave speeds `[S_L, S_R, S_M]` for the 3D Euler equations on the unit normal `n` (Roe-averaged `S_L`, `S_R`).
fn hllc_speeds3(l: &Side3, r: &Side3, n: &Vec3) -> [A; 3] {
    let [rl, ul, vl, zl, pl] = l.w;
    let [rr, ur, vr, zr, pr] = r.w;
    let (unl, unr, cl, cr, el, er) = (&l.un, &r.un, &l.c, &r.c, &l.e, &r.e);

    // Roe averages
    let sql = rl.sqrt();
    let sqr = rr.sqrt();
    let inv = c(1.0) / (&sql + &sqr);
    let uh = (&sql * ul + &sqr * ur) * &inv;
    let vh = (&sql * vl + &sqr * vr) * &inv;
    let wh = (&sql * zl + &sqr * zr) * &inv;
    let hl = (el + pl) / rl;
    let hr = (er + pr) / rr;
    let hh = (&sql * &hl + &sqr * &hr) * &inv;
    let ch2 = c(G1) * (&hh - c(0.5) * (&uh * &uh + &vh * &vh + &wh * &wh));
    let ch = ch2.maximum(c(1e-30)).sqrt();
    let unh = &uh * &n[0] + &vh * &n[1] + &wh * &n[2];

    let sl = (unl - cl).minimum(&unh - &ch);
    let sr = (unr + cr).maximum(&unh + &ch);
    let sl_un = &sl - unl;
    let sr_un = &sr - unr;
    let sm = (pr - pl + rl * unl * &sl_un - rr * unr * &sr_un) / (rl * &sl_un - rr * &sr_un);
    [sl, sr, sm]
}

/// The HLLC flux from the wave speeds of [`hllc_speeds3`]: `F_K + S_K (U*_K - U_K)`, `U*_K = fac [1, u + (S_M - u_n) n, E / rho + (S_M - u_n)
/// (S_M + p / (rho (S_K - u_n)))]`, both tangential velocity components carried passively by the contact.
fn hllc_flux3(l: &Side3, r: &Side3, n: &Vec3, speeds: &[A; 3]) -> Rec3 {
    let [sl, sr, sm] = speeds;
    let sl_un = sl - &l.un;
    let sr_un = sr - &r.un;

    let star = |k: &Side3, s: &A, s_un: &A| -> Rec3 {
        let [r, u, v, ww, p] = k.w;
        let (f, e) = (&k.f, &k.e);
        let fac = r * s_un / (s - sm);
        let sm_un = sm - &k.un;
        [
            &f[0] + s * (&fac - r),
            &f[1] + s * (&fac * (u + &sm_un * &n[0]) - r * u),
            &f[2] + s * (&fac * (v + &sm_un * &n[1]) - r * v),
            &f[3] + s * (&fac * (ww + &sm_un * &n[2]) - r * ww),
            &f[4] + s * (&fac * (e / r + &sm_un * (sm + p / (r * s_un))) - e),
        ]
    };
    let star_l = star(l, sl, &sl_un);
    let star_r = star(r, sr, &sr_un);

    let left = sl.ge(c(0.0));
    let right = sr.le(c(0.0));
    let contact_left = sm.ge(c(0.0));
    rec3(|k| select(&left, &l.f[k], select(&right, &r.f[k], select(&contact_left, &star_l[k], &star_r[k]))))
}

/// Rusanov flux `F = (F_L + F_R) / 2 - s_max (U_R - U_L) / 2`, `s_max = max(|u_nL| + c_L, |u_nR| + c_R)`.
fn rusanov3(l: &Side3, r: &Side3) -> Rec3 {
    let [rl, ul, vl, zl, _] = l.w;
    let [rr, ur, vr, zr, _] = r.w;
    let smax = (l.un.abs() + &l.c).maximum(r.un.abs() + &r.c);
    let dq = [rr - rl, rr * ur - rl * ul, rr * vr - rl * vl, rr * zr - rl * zl, &r.e - &l.e];
    rec3(|k| c(0.5) * (&l.f[k] + &r.f[k]) - c(0.5) * &smax * &dq[k])
}

/// Low-Mach scaling of the face-normal velocity jump: `z = clamp(max(M_L, M_R), 0.05, 1)`, `u_n' = u_avg + z (u_n - u_avg)` on both sides,
/// untouched for `z >= 1`.
fn low_mach_states3(l: Rec3, r: Rec3, n: &Vec3) -> (Rec3, Rec3) {
    let unl = &l[1] * &n[0] + &l[2] * &n[1] + &l[3] * &n[2];
    let unr = &r[1] * &n[0] + &r[2] * &n[1] + &r[3] * &n[2];
    let ml = (&l[1] * &l[1] + &l[2] * &l[2] + &l[3] * &l[3]).sqrt() / (c(GAMMA) * &l[4] / &l[0]).sqrt();
    let mr = (&r[1] * &r[1] + &r[2] * &r[2] + &r[3] * &r[3]).sqrt() / (c(GAMMA) * &r[4] / &r[0]).sqrt();
    let z = ml.maximum(&mr).maximum(c(LOW_MACH_FLOOR)).minimum(c(1.0));
    let active = z.lt(c(1.0));
    let avg = c(0.5) * (&unl + &unr);
    let dl = &avg + &z * (&unl - &avg) - &unl;
    let dr = &avg + &z * (&unr - &avg) - &unr;
    let fix = |w: Rec3, d: &A| -> Rec3 {
        let [rho, u, v, ww, p] = w;
        [rho, select(&active, &u + d * &n[0], &u), select(&active, &v + d * &n[1], &v), select(&active, &ww + d * &n[2], &ww), p]
    };
    (fix(l, &dl), fix(r, &dr))
}

/// Complete face kernel: MUSCL + (low-Mach) + `(1 - theta) HLLC + theta Rusanov`. `split` evaluates the reconstructed states and the shock weight
/// first (`cache`), as `face_flux_lm` does.
fn face_flux_lm3(m2: &Rec3, m1: &Rec3, p0: &Rec3, p1: &Rec3, lim: Limiter, n: &Vec3, rot: &[A; 2], low_mach: bool, split: bool) -> Rec3 {
    let (l, r) = reconstruct3(m2, m1, p0, p1, lim, rot);
    let (l, r) = if low_mach { low_mach_states3(l, r, n) } else { (l, r) };
    let t = shock_weight(&m1[4], &p0[4]);
    let (l, r, t) = if split { (l, r, t).cache() } else { (l, r, t) };
    let (sl, sr) = (Side3::new(&l, n), Side3::new(&r, n));
    let hllc = hllc_flux3(&sl, &sr, n, &hllc_speeds3(&sl, &sr, n));
    let rus = rusanov3(&sl, &sr);
    rec3(|k| (c(1.0) - &t) * &hllc[k] + &t * &rus[k])
}

// ---------------------------------------------------------------------------------------------------------------------
// bc3.rs
// ---------------------------------------------------------------------------------------------------------------------

/// Characteristic far field for the outward unit normal `n` (see `characteristic_ghost`): `R_out = u_n,i + 2 c_i / (g - 1)`, `R_in = u_n,e - 2 c_e /
/// (g - 1)`, `u_n,b = (R_out + R_in) / 2`, `c_b = (g - 1) (R_out - R_in) / 4`; entropy and the tangential velocity vector come from the interior for
/// `u_n,b >= 0`, else from the free stream (state `ref`): `rho_b = rho_ref (c_b / c_ref)^(2 / (g - 1))`, `p_b = rho_b c_b^2 / g`,
/// `v_b = v_ref + (u_n,b - u_n,ref) n`. `M_n <= -1`: free stream, `>= 1`: interior.
fn characteristic_ghost3(fs: &[f64; 5], interior: &Rec3, n: &Vec3) -> Rec3 {
    let ci = (c(GAMMA) * &interior[4] / &interior[0]).sqrt();
    let uni = &interior[1] * &n[0] + &interior[2] * &n[1] + &interior[3] * &n[2];
    let m = &uni / &ci;
    let ce = (GAMMA * fs[4] / fs[0]).sqrt();
    let une = c(fs[1]) * &n[0] + c(fs[2]) * &n[1] + c(fs[3]) * &n[2];
    let r_out = &uni + c(2.0) * &ci / c(G1);
    let r_in = &une - c(2.0 * ce / G1);
    let un_b = c(0.5) * (&r_out + &r_in);
    let c_b = (c(0.25 * G1) * (&r_out - &r_in)).maximum(c(1e-8));
    let from_interior = un_b.ge(c(0.0));
    let rf = [
        select(&from_interior, &interior[0], c(fs[0])),
        select(&from_interior, &interior[1], c(fs[1])),
        select(&from_interior, &interior[2], c(fs[2])),
        select(&from_interior, &interior[3], c(fs[3])),
    ];
    let un_ref = select(&from_interior, &uni, &une);
    let c_ref = select(&from_interior, &ci, c(ce));
    let q = &c_b / &c_ref;
    let q2 = &q * &q;
    let rho = &rf[0] * (&q2 * &q2 * &q);
    let dn = &un_b - &un_ref;
    let riemann = [rho.clone(), &rf[1] + &dn * &n[0], &rf[2] + &dn * &n[1], &rf[3] + &dn * &n[2], &rho * &c_b * &c_b / c(GAMMA)];
    let super_in = m.le(c(-1.0));
    let super_out = m.ge(c(1.0));
    rec3(|k| select(&super_in, c(fs[k]), select(&super_out, &interior[k], &riemann[k])))
}

// ---------------------------------------------------------------------------------------------------------------------
// solver3.rs
// ---------------------------------------------------------------------------------------------------------------------

/// Face geometry of one direction: unit normal, area and the `[sin, cos]` of the azimuth of the centroid (`face_rot`), in the face layout of
/// `Grid3::{iface, jface, kface}`.
struct FaceGeo3 {
    n: Vec3,
    area: A,
    rot: [A; 2],
}

/// The faces of direction `dir` that see a solid cell in their stencil, as the box `lo..hi` of face indices that contains them and the case masks
/// over it (the rules of `Family::fluxes` in `solver3.rs`, with `sm1` / `sp0` = the cell before / after the face is solid):
///
/// ```text
/// z   sm1 && sp0:  flux zero
/// ma  sm1 only:    m1 = R(p0, n),  m2 = R(p1, n)  (mc: p1 solid too: m2 = m1)
/// mb  sp0 only:    p0 = R(m1, n),  p1 = R(m2, n)  (md: m2 solid too: p1 = p0)
/// m2s neither, m2 solid: m2 = R(m1, n of the previous face)       m1s neither, p1 solid: p1 = R(p0, n of the next face)
/// ```
///
/// `R(w, n)` is the reflection about the wall normal `n`; the faces of the box that are none of these keep their own stencil.
struct SolidFaces3 {
    lo: [usize; 3],
    hi: [usize; 3],
    z: Array<bool>,
    ma: Array<bool>,
    mb: Array<bool>,
    mc: Array<bool>,
    md: Array<bool>,
    m2s: Array<bool>,
    m1s: Array<bool>,
    /// The cell before / after the face is a jet cell: its state is both stencil cells on that side (`m1 = m2 = jet`, `p0 = p1 = jet`).
    ja: Array<bool>,
    jb: Array<bool>,
}

/// The motor jet of a run (`Solver3::set_jet`): the jet state `[rho, u, v, w, p]` and `[k, omega]`, the flat indices of the `Part::Jet` cells, their mask over the grid, and the
/// mask of the jet cells that have a fluid cell behind them (`+i`; those take its velocity / temperature gradient as the ghost of the exit face).
struct JetGpu {
    state: crate::solver3::JetState3,
    cells: Vec<usize>,
    mask: Array<bool>,
    behind: Array<bool>,
}

/// Geometry and cell metrics, uploaded once from the grid.
struct Geo3 {
    /// `[i, j, k]` faces: `[ni + 1, nj, nk]`, `[ni, nj + 1, nk]`, `[ni, nj, nk + 1]`.
    f: [FaceGeo3; 3],
    /// `1 / V` (zero in solid cells) and `V` (one in solid cells) per cell.
    inv_v: A,
    vol: A,
    /// Fluid cells (`None` without solid boxes).
    fluid: Option<Array<bool>>,
    /// Outward unit normals of the two ghost layers of the far-field boundaries: `i = 0` (`[2, nj, nk]`), `i = ni` (`[2, nj, nk]`), `j = nj` (`[ni, 2, nk]`).
    imin: Vec3,
    imax: Vec3,
    jmax: Vec3,
    /// Unit normals of the slip-wall faces of `j = 0` (the columns outside the axis runs at `j = 0`), one entry per run of columns `(i0, i1, normals)`
    /// (`[i1 - i0, 2, nk]`, same face in both layers).
    slip: Vec<(usize, usize, Vec3)>,
    /// The axis runs: the ghost rows below the row of a run are the weighted state across the axis (`AxisMap`).
    axis: Vec<AxisGeo3>,
    /// Unit normals of the symmetry planes `k = 0` and `k = nk`: `[ni, nj, 2]`.
    ksym: [Vec3; 2],
    solid_faces: [Option<SolidFaces3>; 3],
}

/// The ghosts of one axis run (the CPU `AxisMapRun`), `[i1 - i0, 2, nk]` arrays, layers outermost first (layer 0 the row `j - 2`, sourced from the row
/// `j + 1`; layer 1 the row `j - 1`, sourced from the row `j`): the flat cell indices (into `[ni, nj, nk]`) of the two sources, their weights, and the
/// 2x2 matrices (`m00 m01 m10 m11`) acting on `(v, w)`. A one-source ghost has weight 1 and 0 and a harmless second index.
struct AxisGeo3 {
    j: usize,
    i0: usize,
    i1: usize,
    idx: [Array<u32>; 2],
    w: [A; 2],
    m: [[A; 4]; 2],
}

impl AxisGeo3 {
    fn new(ctx: &Context, grid: &Grid3, map: &crate::axis3::AxisMapRun) -> Result<AxisGeo3> {
        let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
        let n = map.i1 - map.i0;
        let shape = [n, 2, nk];
        let len = n * 2 * nk;
        let mut idx = [Vec::with_capacity(len), Vec::with_capacity(len)];
        let mut w = [Vec::with_capacity(len), Vec::with_capacity(len)];
        let mut m: [[Vec<f64>; 4]; 2] = std::array::from_fn(|_| std::array::from_fn(|_| Vec::with_capacity(len)));
        for k in 0..nk {
            for layer in 0..2 {
                for i in map.i0..map.i1 {
                    let ghost = &map.ghost[(i - map.i0) * nk + k];
                    let row = map.j + 1 - layer;
                    for s in 0..2 {
                        let src = &ghost.src[s];
                        idx[s].push(((src.k * nj + row) * ni + i) as u32);
                        w[s].push(if ghost.n == 1 && s == 1 { 0.0 } else { src.w });
                        for q in 0..4 {
                            m[s][q].push(src.m[q]);
                        }
                    }
                }
            }
        }
        let arr = |v: &Vec<f64>| up(ctx, v, shape);
        Ok(AxisGeo3 {
            j: map.j,
            i0: map.i0,
            i1: map.i1,
            idx: [Array::from_slice_on(ctx, &idx[0], shape)?, Array::from_slice_on(ctx, &idx[1], shape)?],
            w: [arr(&w[0])?, arr(&w[1])?],
            m: [[arr(&m[0][0])?, arr(&m[0][1])?, arr(&m[0][2])?, arr(&m[0][3])?], [arr(&m[1][0])?, arr(&m[1][1])?, arr(&m[1][2])?, arr(&m[1][3])?]],
        })
    }
}

/// The solid-face case masks of direction `dir` (see [`SolidFaces3`]); `None` if no face sees a solid cell. `solid`: cells that are never updated (walls and
/// dead cells, the CPU `Solver3::solid`); `wall`: the ones that reflect for the outer stencil of a fluid face (the CPU `Solver3::wall`: walls only, a dead
/// cell leaves the padded state there, which holds the axis ghost).
fn solid_faces3(ctx: &Context, solid: &[bool], wall: &[bool], jet: &[bool], dims: [usize; 3], dir: usize) -> Result<Option<SolidFaces3>> {
    const Z: u16 = 1;
    const MA: u16 = 2;
    const MB: u16 = 4;
    const MC: u16 = 8;
    const MD: u16 = 16;
    const M2S: u16 = 32;
    const M1S: u16 = 64;
    // the exit of a motor jet: the cell before (`JA`) / after (`JB`) the face is a `Part::Jet` cell, whose state replaces both stencil cells on its side (the CPU `jet_at`)
    const JA: u16 = 128;
    const JB: u16 = 256;
    let [ni, nj, nk] = dims;
    let at = |flags: &[bool], c: [isize; 3]| {
        c[0] >= 0
            && c[1] >= 0
            && c[2] >= 0
            && (c[0] as usize) < ni
            && (c[1] as usize) < nj
            && (c[2] as usize) < nk
            && flags[(c[2] as usize * nj + c[1] as usize) * ni + c[0] as usize]
    };
    let nf = [ni + (dir == 0) as usize, nj + (dir == 1) as usize, nk + (dir == 2) as usize];
    let mut e = [0isize; 3];
    e[dir] = 1;
    let mut class = vec![0u16; nf[0] * nf[1] * nf[2]];
    let (mut lo, mut hi) = ([usize::MAX; 3], [0usize; 3]);
    for k in 0..nf[2] {
        for j in 0..nf[1] {
            for i in 0..nf[0] {
                let p0 = [i as isize, j as isize, k as isize];
                let off = |s: isize| [p0[0] + s * e[0], p0[1] + s * e[1], p0[2] + s * e[2]];
                // `solid` decides the wall faces and the stencil next to them (`Family::fluxes`: `is_solid`), `wall` the outer stencil cell of a fluid face
                let (s_m2, s_m1, s_p0, s_p1) = (at(solid, off(-2)), at(solid, off(-1)), at(solid, p0), at(solid, off(1)));
                let (w_m2, w_p1) = (at(wall, off(-2)), at(wall, off(1)));
                if !(s_m1 || s_p0 || w_m2 || w_p1) {
                    continue;
                }
                let mut bits: u16 = 0;
                if s_m1 && s_p0 {
                    bits |= Z;
                } else if s_m1 {
                    bits |= MA | if s_p1 { MC } else { 0 } | if at(jet, off(-1)) { JA } else { 0 };
                } else if s_p0 {
                    bits |= MB | if s_m2 { MD } else { 0 } | if at(jet, p0) { JB } else { 0 };
                } else {
                    bits |= if w_m2 { M2S } else { 0 } | if w_p1 { M1S } else { 0 };
                }
                class[(k * nf[1] + j) * nf[0] + i] = bits;
                for (a, x) in [i, j, k].into_iter().enumerate() {
                    lo[a] = lo[a].min(x);
                    hi[a] = hi[a].max(x + 1);
                }
            }
        }
    }
    if hi[0] == 0 {
        return Ok(None);
    }
    let shape = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let mask = |bit: u16| {
        let mut v = Vec::with_capacity(shape[0] * shape[1] * shape[2]);
        for k in lo[2]..hi[2] {
            for j in lo[1]..hi[1] {
                for i in lo[0]..hi[0] {
                    v.push(class[(k * nf[1] + j) * nf[0] + i] & bit != 0);
                }
            }
        }
        Array::from_slice_on(ctx, &v, shape)
    };
    Ok(Some(SolidFaces3 {
        lo,
        hi,
        z: mask(Z)?,
        ma: mask(MA)?,
        mb: mask(MB)?,
        mc: mask(MC)?,
        md: mask(MD)?,
        m2s: mask(M2S)?,
        m1s: mask(M1S)?,
        ja: mask(JA)?,
        jb: mask(JB)?,
    }))
}

/// Device arrays of the force integration, one entry per wall face (`[faces]`): the cells `a`, `b` of the wall-pressure extrapolation, its coefficient,
/// the weights of `Fx`, `Fz`, `Mz` and the class masks (all faces, and the faces of the monitor's cut).
struct ForceGeo3 {
    ia: Array<u32>,
    ib: Array<u32>,
    k: A,
    cx: A,
    cz: A,
    cm: A,
    class: Vec<Array<bool>>,
    cut_class: Vec<Array<bool>>,
}

/// The force monitor's faces: the pressure faces with the weights of `Fx`, `Fz`, `Mz` of the monitored classes (zero elsewhere) and the no-slip faces of the monitored
/// classes per family (`-sigma area` and `-sigma area x_c`).
struct MonitorGeo3 {
    ia: Array<u32>,
    ib: Array<u32>,
    k: A,
    wx: A,
    wz: A,
    wm: A,
    fam: [Option<MonitorFam3>; 3],
}

struct MonitorFam3 {
    idx: Array<u32>,
    w: A,
    wm: A,
}

/// The loop state at one iteration: see [`LazySolver3::snapshot`].
#[derive(Clone)]
pub struct Snapshot3 {
    state: State3,
    iter: usize,
}

/// The time-step cap of a wedge's axis cells (`WEDGE_AXIS_DT r_c`, in the solver's precision and in f32 for the implicit left-hand side).
struct AxisDt3 {
    mask: Array<bool>,
    cap: A,
    cap_f: Array<f32>,
}

/// A loop traced by `luma::compile_until_all` and the setting it was traced for.
struct CompiledLoop3 {
    key: (Limiter, Staging, [bool; 4], usize),
    /// The steps it was compiled for (shorter runs use `run_steps`).
    max: usize,
    run: luma::CompiledUntil<State3>,
}

/// The solver: state, geometry and the loops that advance it. Mirrors `crate::solver3::Solver3` with `Scheme3::Explicit` (`eval_residual` and
/// `rk3_step`) or `Scheme3::Implicit` (`implicit3.rs`).
pub struct LazySolver3 {
    ctx: Context,
    pub ni: usize,
    pub nj: usize,
    pub nk: usize,
    cfg: Config3,
    /// How the evaluation is ordered (see [`Staging`]).
    pub staging: Staging,
    geo: Geo3,
    state: Option<State3>,
    pub iter: usize,
    /// Wedge-mode switches of the implicit scheme (on for a [`crate::grid3::Symmetry::Wedge`] grid, as on the CPU `Solver3::is_wedge`; `GpuSolver3::set_wedge_parts`
    /// changes them for experiments). `skip_k`: no k factor in the line solves. `no_dt_k`: no `L_k` in the pseudo-time step. `k_wall_flux`: the flux through the
    /// two k planes is the pressure of the cell, `[0, p n, 0]` (the hoop term of the axisymmetric equations), not the Riemann flux between the cell and its
    /// mirror image. `k_src`: the point-implicit pressure Jacobian of that flux joins the diagonal of the j factor (as the 2D `cell_blocks` `src`).
    pub skip_k: bool,
    pub no_dt_k: bool,
    pub k_wall_flux: bool,
    pub k_src: bool,
    /// The f32 geometry of the implicit left-hand side.
    lg: LGeo3,
    /// The wall faces of the on-device force integration (`set_wall_faces`).
    forces: Option<ForceGeo3>,
    /// The viscous terms (`viscous3.rs`): the host-built face data; `None` for the Euler equations.
    visc: Option<VGeo3>,
    /// The no-slip faces of the viscous force reduction.
    traction: Option<TractionGeo3>,
    /// The SST model of a viscous run (`rans3.rs`); `None`: laminar or Euler.
    turb: Option<TurbRun3>,
    /// The per-cell update mask on the host (set-up of the turbulence state).
    solid_host: Vec<bool>,
    /// Back pressure of the `i = ni` static-pressure outflow of a viscous run (`RansOpts3::pressure_outflow`); `None`: the characteristic ghost.
    p_out: Option<f64>,
    /// The runs of columns `i` of `j = 0` that are the no-slip body surface of a viscous run (`Grid3::wall_j0`; the axis columns excluded): the ghost layers there are the
    /// no-slip ghost, not the mirror image (`Solver3::fill_ghosts`).
    wall_j0: Vec<std::ops::Range<usize>>,
    /// The same columns as a mask `[ni, 1, nk]` for the ghost Jacobian of the left-hand side (`None` without any).
    wall_j0_cols: Option<Array<bool>>,
    /// `(sin, cos)` of half the wedge angle in a viscous wedge run: the azimuthal momentum of the residual is projected out (`Solver3::add_viscous_residual`).
    swirl: Option<(f64, f64)>,
    /// The motor jet: its state and the jet cells (`None`: no jet).
    jet: Option<JetGpu>,
    /// The force monitor of the loop (`set_force_monitor`).
    monitor: Option<MonitorGeo3>,
    /// The loop of [`LazySolver3::run_monitored`] traced once for its key (limiter, staging, wedge switches, sync interval).
    compiled: Option<CompiledLoop3>,
    /// The mask of the density residual's cells (fluid cells off the axis runs; `None` when the solid cells, whose residual is zero, are the only ones left out) and their count.
    norm: Option<Array<bool>>,
    n_norm: usize,
    /// The axis cells of a wedge and `WEDGE_AXIS_DT r_c` of every cell: the time step there is at most `cap / c`.
    axis_dt: Option<AxisDt3>,
}

impl LazySolver3 {
    /// Uploads the geometry (rounded from the grid's f64 metrics to `S`) and starts from the free stream.
    pub fn new(ctx: &Context, grid: &Grid3, cfg: &Config3) -> Result<LazySolver3> {
        Self::new_with_jet(ctx, grid, cfg, None)
    }

    /// [`LazySolver3::new`] for a grid whose `Part::Jet` cells hold the jet state `jet` (`Solver3::set_jet`): they are solid but no wall.
    pub fn new_with_jet(ctx: &Context, grid: &Grid3, cfg: &Config3, jet: Option<&crate::solver3::JetState3>) -> Result<LazySolver3> {
        let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
        let nc = ni * nj * nk;
        // never updated: walls and dead cells (`Grid3::part != PART_FLUID`, the CPU `Solver3::solid`); `wall`: the ones that reflect (`Solver3::wall`); a jet cell is none
        let solid: Vec<bool> = grid.part.iter().map(|&p| p != crate::grid3::PART_FLUID).collect();
        let jet_mask: Vec<bool> = (0..nc).map(|c| jet.is_some() && crate::viscous3::is_jet_cell(grid, c)).collect();
        let wall: Vec<bool> = (0..nc).map(|c| grid.part[c] != crate::grid3::PART_FLUID && grid.part[c] != crate::grid3::PART_DEAD && !jet_mask[c]).collect();
        let wedge = grid.sym == crate::grid3::Symmetry::Wedge;
        let n_active = solid.iter().filter(|s| !**s).count();
        let has_solid = n_active < nc;
        // the residual norm leaves the cells of the axis runs out; a wedge limits the time step of its axis cells (`Solver3::in_norm`, `WEDGE_AXIS_DT`)
        let axis: Vec<bool> = (0..nc).map(|c| grid.is_axis(c % ni, (c / ni) % nj)).collect();
        let any_axis = axis.iter().any(|a| *a);
        let n_norm = (0..nc).filter(|&c| !solid[c] && !axis[c]).count();
        let norm = if any_axis { Some(Array::from_slice_on(ctx, &(0..nc).map(|c| !solid[c] && !axis[c]).collect::<Vec<_>>(), [ni, nj, nk])?) } else { None };
        let axis_dt = if wedge && any_axis {
            let cap: Vec<f64> = (0..nc).map(|c| crate::solver3::WEDGE_AXIS_DT * grid.xc[c][1].hypot(grid.xc[c][2])).collect();
            Some(AxisDt3 {
                mask: Array::from_slice_on(ctx, &axis, [ni, nj, nk])?,
                cap: up(ctx, &cap, [ni, nj, nk])?,
                cap_f: Array::from_slice_on(ctx, &cap.iter().map(|&x| x as f32).collect::<Vec<_>>(), [ni, nj, nk])?,
            })
        } else {
            None
        };

        let face_geo = |faces: &[Face3], shape: [usize; 3]| -> Result<FaceGeo3> {
            let get = |g: &dyn Fn(&Face3) -> f64| up(ctx, &faces.iter().map(g).collect::<Vec<_>>(), shape);
            let rots: Vec<[f64; 2]> = faces.iter().map(|f| face_rot(f.c)).collect();
            let rot = |q: usize| up(ctx, &rots.iter().map(|r| r[q]).collect::<Vec<_>>(), shape);
            Ok(FaceGeo3 { n: [get(&|f| f.n[0])?, get(&|f| f.n[1])?, get(&|f| f.n[2])?], area: get(&|f| f.area)?, rot: [rot(0)?, rot(1)?] })
        };
        // normals of a list of faces (in the order of the array to build), each multiplied by `sign`
        let normals = |faces: Vec<&Face3>, sign: f64, shape: [usize; 3]| -> Result<Vec3> {
            let get = |q: usize| up(ctx, &faces.iter().map(|f| sign * f.n[q]).collect::<Vec<_>>(), shape);
            Ok([get(0)?, get(1)?, get(2)?])
        };
        // i boundaries `[layer, j, k]`
        let iface_at =
            |i: usize| -> Vec<&Face3> { (0..nk).flat_map(|k| (0..nj).flat_map(move |j| [(k * nj + j) * (ni + 1) + i; 2])).map(|q| &grid.iface[q]).collect() };
        let imin = normals(iface_at(0), -1.0, [2, nj, nk])?;
        let imax = normals(iface_at(ni), 1.0, [2, nj, nk])?;
        // j boundaries `[i, layer, k]`
        let jface_row = |j: usize, i0: usize, i1: usize| -> Vec<&Face3> {
            (0..nk).flat_map(|k| (0..2).flat_map(move |_| (i0..i1).map(move |i| (k * (nj + 1) + j) * ni + i))).map(|q| &grid.jface[q]).collect()
        };
        let jmax = normals(jface_row(nj, 0, ni), 1.0, [ni, 2, nk])?;
        // the columns of `j = 0` outside the axis runs at `j = 0` are the slip wall: one entry per run of such columns
        let mut slip = Vec::new();
        let mut i0 = 0;
        while i0 < ni {
            if grid.is_axis(i0, 0) {
                i0 += 1;
                continue;
            }
            let mut i1 = i0;
            while i1 < ni && !grid.is_axis(i1, 0) {
                i1 += 1;
            }
            slip.push((i0, i1, normals(jface_row(0, i0, i1), 1.0, [i1 - i0, 2, nk])?));
            i0 = i1;
        }
        // symmetry planes `[i, j, layer]`
        let kplane = |k: usize| -> Vec<&Face3> {
            (0..2).flat_map(|_| (0..nj).flat_map(move |j| (0..ni).map(move |i| (k * nj + j) * ni + i))).map(|q| &grid.kface[q]).collect()
        };
        let ksym = [normals(kplane(0), 1.0, [ni, nj, 2])?, normals(kplane(nk), 1.0, [ni, nj, 2])?];
        // the axis runs: the ghost rows below the row of a run are the weighted state across the axis
        let axis = crate::axis3::AxisMap::new(grid).runs.iter().map(|run| AxisGeo3::new(ctx, grid, run)).collect::<Result<Vec<_>>>()?;
        let geo = Geo3 {
            f: [face_geo(&grid.iface, [ni + 1, nj, nk])?, face_geo(&grid.jface, [ni, nj + 1, nk])?, face_geo(&grid.kface, [ni, nj, nk + 1])?],
            inv_v: up(ctx, &grid.vol.iter().zip(&solid).map(|(v, s)| if *s { 0.0 } else { 1.0 / v }).collect::<Vec<_>>(), [ni, nj, nk])?,
            vol: up(ctx, &grid.vol.iter().zip(&solid).map(|(v, s)| if *s { 1.0 } else { *v }).collect::<Vec<_>>(), [ni, nj, nk])?,
            fluid: if has_solid { Some(Array::from_slice_on(ctx, &solid.iter().map(|s| !s).collect::<Vec<_>>(), [ni, nj, nk])?) } else { None },
            imin,
            imax,
            jmax,
            slip,
            axis,
            ksym,
            solid_faces: if has_solid {
                [
                    solid_faces3(ctx, &solid, &wall, &jet_mask, [ni, nj, nk], 0)?,
                    solid_faces3(ctx, &solid, &wall, &jet_mask, [ni, nj, nk], 1)?,
                    solid_faces3(ctx, &solid, &wall, &jet_mask, [ni, nj, nk], 2)?,
                ]
            } else {
                [None, None, None]
            },
        };
        let lg = LGeo3::new(ctx, grid, &solid, &jet_mask)?;
        let jet_gpu = match jet {
            Some(j) => {
                let cells: Vec<usize> = (0..nc).filter(|&c| jet_mask[c]).collect();
                let behind: Vec<bool> = (0..nc).map(|c| jet_mask[c] && c % ni + 1 < ni && !solid[c + 1]).collect();
                Some(JetGpu {
                    state: *j,
                    cells,
                    mask: Array::from_slice_on(ctx, &jet_mask, [ni, nj, nk])?,
                    behind: Array::from_slice_on(ctx, &behind, [ni, nj, nk])?,
                })
            }
            None => None,
        };
        let mut s = LazySolver3 {
            ctx: ctx.clone(),
            ni,
            nj,
            nk,
            cfg: cfg.clone(),
            staging: DEFAULT_STAGING,
            geo,
            state: None,
            iter: 0,
            skip_k: wedge,
            no_dt_k: wedge,
            k_wall_flux: wedge,
            k_src: wedge,
            lg,
            forces: None,
            visc: None,
            traction: None,
            turb: None,
            solid_host: solid,
            p_out: None,
            wall_j0: Vec::new(),
            wall_j0_cols: None,
            swirl: None,
            jet: jet_gpu,
            monitor: None,
            compiled: None,
            norm,
            n_norm,
            axis_dt,
        };
        let q = prim_to_cons3(&cfg.freestream);
        s.set_state(&vec![q; nc])?;
        Ok(s)
    }

    /// A lazy solver in the state of a host solver: grid, config, conserved state and iteration count.
    pub fn from_host(ctx: &Context, s: &Solver3) -> Result<LazySolver3> {
        let jet = s.jet();
        let mut l = LazySolver3::new_with_jet(ctx, &s.grid, &s.cfg, jet.as_ref())?;
        l.set_viscous_from_host(s)?;
        if l.visc.is_some() && matches!(s.cfg.scheme, Scheme3::Implicit(p) if p.line_solve == crate::implicit_params::LineSolve3::DiagonalCorrected) {
            return Err(luma::LumaError::Unsupported("the diagonal line solve of a viscous run is not ported (use LineSolve3::Block)".into()));
        }
        if matches!(s.cfg.scheme, Scheme3::Implicit(p) if p.low_mach_precond && p.line_solve == crate::implicit_params::LineSolve3::DiagonalCorrected) {
            return Err(luma::LumaError::Unsupported("the diagonal line solve with the low-Mach preconditioning is not ported (it stalls on the CPU too: use LineSolve3::Block)".into()));
        }
        l.set_state(&s.u)?;
        l.iter = s.iter;
        Ok(l)
    }

    pub fn config(&self) -> &Config3 {
        &self.cfg
    }

    /// Replaces the configuration (free stream, CFL, limiter, `first_order_iters`, low-Mach flag, scheme) without touching the state or the geometry:
    /// the next run reads it when it traces its step. Changing the scheme (or its parameters) restarts the CFL ramp; a change of the free stream
    /// does not ([`LazySolver3::restart_ramp`]).
    pub fn set_config(&mut self, cfg: &Config3) -> Result<()> {
        let scheme_changed = cfg.scheme != self.cfg.scheme;
        self.compiled = None;
        self.cfg = cfg.clone();
        if scheme_changed {
            self.restart_ramp()?;
        }
        Ok(())
    }

    /// The convergence scheme.
    pub fn scheme(&self) -> Scheme3 {
        self.cfg.scheme
    }

    /// CFL of the first iteration after a restart of the ramp: `min(cfl0, cfl_max)` for the implicit scheme, the constant `Config3::cfl` of the explicit one.
    fn cfl_start(&self) -> f64 {
        match self.cfg.scheme {
            Scheme3::Implicit(p) => p.cfl0.min(p.cfl_max),
            Scheme3::Explicit => self.cfg.cfl,
        }
    }

    /// Restarts the CFL ramp of the implicit scheme at `cfl0` (call at the start of a run and after a change of the free stream; the loop state of
    /// the explicit scheme carries the constant CFL).
    pub fn restart_ramp(&mut self) -> Result<()> {
        let cfl = up(&self.ctx, &[self.cfl_start()], 1)?;
        if let Some(s) = &mut self.state {
            s[6] = cfl;
        }
        Ok(())
    }

    /// A copy of the loop state to return to ([`LazySolver3::restore`]). The arrays are immutable values: the copy shares the device buffers, nothing moves or is allocated
    /// but the references (a step makes new arrays, it does not write into the old ones).
    pub fn snapshot(&self) -> Result<Snapshot3> {
        Ok(Snapshot3 { state: self.state.clone().ok_or_else(no_state)?, iter: self.iter })
    }

    /// Returns to `snap` (the state, the CFL of its ramp and the iteration count); with `cfl_cap` the CFL of the state is at most that (a controller that lowers `cfl_max`
    /// after a failure: [`LazySolver3::set_config`] first, then this).
    pub fn restore(&mut self, snap: &Snapshot3, cfl_cap: Option<f64>) -> Result<()> {
        let mut state = snap.state.clone();
        if let Some(cap) = cfl_cap {
            let cfl = to_f64(&state[6])?[0].min(cap);
            state[6] = up(&self.ctx, &[cfl], 1)?;
        }
        self.state = Some(state);
        self.iter = snap.iter;
        Ok(())
    }

    /// Overwrites the conserved state and restarts the iteration counter and the CFL ramp (`k`, `omega`, `mu_t` of an SST run at the free stream).
    pub fn set_state(&mut self, u: &[Cons3]) -> Result<()> {
        self.set_state_turb(u, None)
    }

    /// [`LazySolver3::set_state`] with the `[k, omega]` of every cell given (`Some`; `mu_t = min(rho k / omega, 1e5 / re_c)`, the host init's value for a cell without strain and
    /// wall distance). Solid cells and jet cells are as in `set_state`; `None` is the free stream.
    pub fn set_state_turb(&mut self, u: &[Cons3], turb_in: Option<&[[f64; 2]]>) -> Result<()> {
        // the jet cells hold the jet state whatever the caller passes (a reset to the free stream, a state of another Mach number)
        let mut u = u.to_vec();
        if let Some(j) = &self.jet {
            let q = prim_to_cons3(&j.state.state);
            for &c in &j.cells {
                u[c] = q;
            }
        }
        let mut s = upload_state3(&self.ctx, &u, [self.ni, self.nj, self.nk], self.cfl_start())?;
        // an SST run restarts `k`, `omega`, `mu_t` at the free stream (the host `turbulence3::init`) or at the given scalars; the other runs carry three one-element placeholders
        if let Some(t) = &self.turb {
            let (mut turb, mut mu_t) = t.init_host(&self.solid_host, self.cfg.freestream[0]);
            if let Some(given) = turb_in {
                let cap = 1e5 / t.re_c;
                for c in 0..turb.len() {
                    if !self.solid_host[c] {
                        turb[c] = given[c];
                        mu_t[c] = (u[c][0] * given[c][0] / given[c][1]).min(cap) * t.gamma_host.get(c).copied().unwrap_or(1.0);
                    }
                }
            }
            if let Some(j) = &self.jet {
                for &c in &j.cells {
                    turb[c] = j.state.turb;
                    mu_t[c] = j.state.state[0] * j.state.turb[0] / j.state.turb[1];
                }
            }
            let shape = [self.ni, self.nj, self.nk];
            s[7] = up(&self.ctx, &turb.iter().map(|x| x[0]).collect::<Vec<_>>(), shape)?;
            s[8] = up(&self.ctx, &turb.iter().map(|x| x[1]).collect::<Vec<_>>(), shape)?;
            s[9] = up(&self.ctx, &mu_t, shape)?;
        }
        self.state = Some(s);
        self.iter = 0;
        Ok(())
    }

    /// The conserved state on the host, `ni * nj * nk` in the order of `Grid3::cell`.
    pub fn state_f64(&self) -> Result<Vec<Cons3>> {
        download_state3(self.state.as_ref().ok_or_else(no_state)?)
    }

    /// Uploads the wall faces of the force integration (see [`crate::gpu::WallFace3`]) for `n_classes` classes.
    pub fn set_wall_faces(&mut self, faces: &[crate::gpu::WallFace3], n_classes: usize) -> Result<()> {
        let ctx = &self.ctx;
        let n = faces.len().max(1);
        let pad = |v: Vec<f64>| -> Vec<f64> {
            let mut v = v;
            v.resize(n, 0.0);
            v
        };
        let idx = |f: &dyn Fn(&crate::gpu::WallFace3) -> usize| -> Result<Array<u32>> {
            let mut v: Vec<u32> = faces.iter().map(|x| f(x) as u32).collect();
            v.resize(n, 0);
            Array::from_slice_on(ctx, &v, [n])
        };
        let val = |f: &dyn Fn(&crate::gpu::WallFace3) -> f64| up(ctx, &pad(faces.iter().map(f).collect()), [n]);
        let mask = |f: &dyn Fn(&crate::gpu::WallFace3) -> bool| -> Result<Array<bool>> {
            let mut v: Vec<bool> = faces.iter().map(f).collect();
            v.resize(n, false);
            Array::from_slice_on(ctx, &v, [n])
        };
        let class = (0..n_classes).map(|c| mask(&|x| x.class == c)).collect::<Result<Vec<_>>>()?;
        let cut_class = (0..n_classes).map(|c| mask(&|x| x.class == c && x.cut)).collect::<Result<Vec<_>>>()?;
        self.forces = Some(ForceGeo3 {
            ia: idx(&|x| x.a)?,
            ib: idx(&|x| x.b.unwrap_or(x.a))?,
            k: val(&|x| if x.b.is_some() { x.k } else { 0.0 })?,
            cx: val(&|x| x.cx)?,
            cz: val(&|x| x.cz)?,
            cm: val(&|x| x.cm)?,
            class,
            cut_class,
        });
        Ok(())
    }

    /// The force monitor: the forces `[Fx, Fz, Mz]` of the classes `keep` (pressure on the cut faces, the shear of the no-slip faces of the second set of traction
    /// classes: the `wall_forces().1` sums of the runner, classes not kept left out) summed in every step of the loop from the pressure and the viscous face fluxes
    /// the step evaluates anyway, and recorded in the history of [`LazySolver3::run_monitored`]. `faces` / `tfaces` are the lists given to `set_wall_faces` /
    /// `set_traction_faces` (the classes `n..2n` of the traction faces are the cut copies of the classes `0..n`, `n = keep.len()`).
    pub fn set_force_monitor(&mut self, faces: &[crate::gpu::WallFace3], tfaces: &[crate::gpu::TractionFace3], keep: &[bool]) -> Result<()> {
        self.compiled = None;
        let ctx = &self.ctx;
        let n = faces.len().max(1);
        let sel = |f: &crate::gpu::WallFace3| f64::from(u8::from(f.cut && keep[f.class]));
        let padded = |g: &dyn Fn(&crate::gpu::WallFace3) -> f64| -> Vec<f64> {
            let mut v: Vec<f64> = faces.iter().map(g).collect();
            v.resize(n, 0.0);
            v
        };
        let mut idx_a: Vec<u32> = faces.iter().map(|f| f.a as u32).collect();
        let mut idx_b: Vec<u32> = faces.iter().map(|f| f.b.unwrap_or(f.a) as u32).collect();
        idx_a.resize(n, 0);
        idx_b.resize(n, 0);
        let mut fam: [Option<MonitorFam3>; 3] = [None, None, None];
        for (dir, slot) in fam.iter_mut().enumerate() {
            let list: Vec<&crate::gpu::TractionFace3> = tfaces.iter().filter(|f| f.dir == dir && f.class >= keep.len() && keep[f.class - keep.len()]).collect();
            if list.is_empty() {
                continue;
            }
            let m = list.len();
            let w: Vec<f64> = list.iter().map(|f| -f.sigma * f.area).collect();
            *slot = Some(MonitorFam3 {
                idx: Array::from_slice_on(ctx, &list.iter().map(|f| f.q as u32).collect::<Vec<_>>(), [m])?,
                wm: up(ctx, &list.iter().zip(&w).map(|(f, w)| w * f.cx).collect::<Vec<_>>(), [m])?,
                w: up(ctx, &w, [m])?,
            });
        }
        self.monitor = Some(MonitorGeo3 {
            ia: Array::from_slice_on(ctx, &idx_a, [n])?,
            ib: Array::from_slice_on(ctx, &idx_b, [n])?,
            k: up(ctx, &padded(&|f| if f.b.is_some() { f.k } else { 0.0 }), [n])?,
            wx: up(ctx, &padded(&|f| sel(f) * f.cx), [n])?,
            wz: up(ctx, &padded(&|f| sel(f) * f.cz), [n])?,
            wm: up(ctx, &padded(&|f| sel(f) * f.cm), [n])?,
            fam,
        });
        Ok(())
    }

    /// The force monitor of the state whose primitive pressure is `pressure` and whose viscous face fluxes are `fv` (`None` without a monitor): `[Fx, Fz, Mz]`.
    fn force_monitor3(&self, pressure: &A, fv: Option<&[Vec4; 3]>) -> Option<A> {
        let m = self.monitor.as_ref()?;
        let (pa, pb) = (pressure.take(&m.ia), pressure.take(&m.ib));
        let d = &pa + &m.k * (&pa - &pb) - c(self.cfg.freestream[4]);
        let (mut x, mut z, mut mm) = (vec![&d * &m.wx], vec![&d * &m.wz], vec![&d * &m.wm]);
        if let Some(fv) = fv {
            for (dir, f) in m.fam.iter().enumerate() {
                let Some(f) = f else { continue };
                let (fx, fz) = (fv[dir][0].take(&f.idx), fv[dir][2].take(&f.idx));
                x.push(&fx * &f.w);
                mm.push(&fz * &f.wm);
                z.push(&fz * &f.w);
            }
        }
        let sum = |v: Vec<A>| luma::concat_slice(0, &v).sum().reshape([1]);
        Some(luma::concat_slice(0, &[sum(x), sum(z), sum(mm)]))
    }
    /// The smallest density and pressure of the state, `[rho_min, p_min]` over every cell (the solid ones hold the free stream), reduced on the device: two numbers are
    /// downloaded, the state is not. The health check of the CFL controller ([`crate::gpu::GpuSolver3::state_extrema`]).
    pub fn state_extrema(&self) -> Result<[f64; 2]> {
        let s = self.state.as_ref().ok_or_else(no_state)?;
        let w = cons_to_prim3(unpack3(s));
        let v = to_f64(&luma::concat_slice(0, &[w[0].min().reshape([1]), w[4].min().reshape([1])]))?;
        Ok([v[0], v[1]])
    }

    /// Pressure forces of the wall faces by class, `[Fx, Fz, Mz]` as in the CPU `wall_forces` (not scaled to the whole body), reduced on the device and
    /// read back as `2 n_classes` triples: the classes over all faces, then over the faces flagged `cut` (the monitor's `x <= x_max`). One download of
    /// `6 n_classes` numbers; the state itself is not downloaded. The wall pressure of a face is `p_a + k (p_a - p_b)` (the extrapolation of the two first cells).
    pub fn wall_forces(&self, p_inf: f64) -> Result<Vec<[f64; 3]>> {
        let fg = self.forces.as_ref().ok_or_else(|| luma::LumaError::Unsupported("wall faces are not set".into()))?;
        let s = self.state.as_ref().ok_or_else(no_state)?;
        let u = unpack3(s);
        let pall = cons_to_prim3(u)[4].clone();
        let (pa, pb) = (pall.take(&fg.ia), pall.take(&fg.ib));
        let pw = &pa + &fg.k * (&pa - &pb);
        let d = pw - c(p_inf);
        let comps = [&d * &fg.cx, &d * &fg.cz, &d * &fg.cm];
        let mut sums: Vec<A> = Vec::new();
        for m in fg.class.iter().chain(&fg.cut_class) {
            for comp in &comps {
                sums.push(comp.sum_where(m).reshape([1]));
            }
        }
        if sums.is_empty() {
            return Ok(Vec::new());
        }
        let flat = to_f64(&luma::concat_slice(0, &sums))?;
        Ok(flat.chunks(3).map(|t| [t[0], t[1], t[2]]).collect())
    }

    pub fn set_low_mach(&mut self, on: bool) {
        self.cfg.low_mach = on;
        self.compiled = None;
    }

    /// `sqrt(sum (d rho / dt)^2 / N)`, `N` the number of fluid cells off the axis runs, the cells of an axis run left out of the sum (`Solver3::density_residual`).
    fn residual_norm3(&self, r0: &A) -> A {
        let sq = r0 * r0;
        let sq = match &self.norm {
            Some(m) => select(m, sq, c(0.0)),
            None => sq,
        };
        (sq.sum() / c(self.n_norm as f64)).sqrt()
    }

    /// Writes the ghost layers into the padded primitives `w` with region assigns (as `Solver3::fill_ghosts`): `p` is the unpadded primitive state.
    /// `w` is a lazy expression, so nothing is launched: `w` becomes the padded state with the ghost expressions in place, and fuses into whatever
    /// reads it. Layers are outermost first, as the cells they stand for. There are no corner ghosts (every stencil is one-dimensional).
    fn assign_ghosts3(&self, p: &Rec3, w: &mut Rec3) {
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let g = &self.geo;
        let fs = self.cfg.freestream;
        let ok = "ghost layer assign: ghost shapes fixed by the grid";

        // i = 0 and i = ni: characteristic far field, both layers the same state
        let edge_lo = rec3(|f| p[f].pad([(2, 0)], PadMode::Edge).slice(0, 0..2));
        w.slice_mut(0, ..2).slice(1, 2..nj + 2).slice(2, 2..nk + 2).assign(characteristic_ghost3(&fs, &edge_lo, &g.imin)).expect(ok);
        let edge_hi = rec3(|f| p[f].pad([(0, 2)], PadMode::Edge).slice(0, ni..ni + 2));
        // a viscous run: the static-pressure outflow at `i = ni` (`bc3::pressure_outflow_ghost3`; the characteristic ghost would impose the free-stream normal
        // velocity on the boundary layer), subsonic faces take `rho, u, v, w` from the cell and `p = p_back`, supersonic faces extrapolate
        let ghost_hi = match self.p_out {
            Some(pb) => {
                let un = &edge_hi[1] * &g.imax[0] + &edge_hi[2] * &g.imax[1] + &edge_hi[3] * &g.imax[2];
                let ci = (c(GAMMA) * &edge_hi[4] / &edge_hi[0]).sqrt();
                let sub = (&un / &ci).lt(c(1.0));
                rec3(|k| if k == 4 { select(&sub, c(pb), &edge_hi[4]) } else { edge_hi[k].clone() })
            }
            None => characteristic_ghost3(&fs, &edge_hi, &g.imax),
        };
        w.slice_mut(0, ni + 2..).slice(1, 2..nj + 2).slice(2, 2..nk + 2).assign(ghost_hi).expect(ok);

        // j = nj: characteristic far field
        let edge_top = rec3(|f| p[f].pad([(0, 0), (0, 2)], PadMode::Edge).slice(1, nj..nj + 2));
        w.slice_mut(1, nj + 2..).slice(0, 2..ni + 2).slice(2, 2..nk + 2).assign(characteristic_ghost3(&fs, &edge_top, &g.jmax)).expect(ok);

        // the axis runs (`Solver3::fill_ghosts`): ghost row `j - m` of a run at row `j` is `sum_s w_s (m_s . (v, w))` of the cell `(i, j + m - 1, k_s)`, the other
        // primitives copied (`AxisGhost::state`)
        for run in &g.axis {
            let (a, b) = (p.take(&run.idx[0]), p.take(&run.idx[1]));
            let (w0, w1) = (&run.w[0], &run.w[1]);
            let (m0, m1) = (&run.m[0], &run.m[1]);
            let mix = |f: usize| w0 * &a[f] + w1 * &b[f];
            let ghost: Rec3 = [
                mix(0),
                mix(1),
                w0 * (&m0[0] * &a[2] + &m0[1] * &a[3]) + w1 * (&m1[0] * &b[2] + &m1[1] * &b[3]),
                w0 * (&m0[2] * &a[2] + &m0[3] * &a[3]) + w1 * (&m1[2] * &b[2] + &m1[3] * &b[3]),
                mix(4),
            ];
            w.slice_mut(0, 2 + run.i0..2 + run.i1).slice(1, run.j..run.j + 2).slice(2, 2..nk + 2).assign(ghost).expect(ok);
        }
        // the rest of j = 0: slip wall, except the columns of `Grid3::wall_j0` of a viscous run (the body surface: the no-slip ghost `(rho, -u, -v, -w, p)`)
        for (i0, i1, n) in &g.slip {
            let mut cuts = vec![*i0, *i1];
            for r in &self.wall_j0 {
                cuts.extend([r.start, r.end].into_iter().filter(|b| b > i0 && b < i1));
            }
            cuts.sort_unstable();
            cuts.dedup();
            for seg in cuts.windows(2) {
                let (a, b) = (seg[0], seg[1]);
                let mirror = rec3(|f| p[f].pad([(0, 0), (2, 0)], PadMode::Mirror).slice(1, 0..2).slice(0, a..b));
                let ghost = if self.visc.is_some() && self.wall_j0.iter().any(|r| r.contains(&a)) {
                    [mirror[0].clone(), -&mirror[1], -&mirror[2], -&mirror[3], mirror[4].clone()]
                } else {
                    let nn: Vec3 = std::array::from_fn(|q| n[q].slice(0, a - i0..b - i0));
                    reflect3(&mirror, &nn)
                };
                w.slice_mut(0, 2 + a..2 + b).slice(1, 0..2).slice(2, 2..nk + 2).assign(ghost).expect(ok);
            }
        }

        // k = 0 and k = nk: symmetry planes
        if nk == 1 {
            // One cell between the two planes. The CPU fills layer 1 of both planes from the cell, then layer 2 from "the cell `m - 1` / `nk - m`", which
            // for `nk = 1` is the first ghost layer of the other plane: layer 2 of a plane is the reflection of the other plane's layer 1 (a double
            // reflection of the cell). `PadMode::Mirror` cannot say that (a mirror of a one-cell axis repeats the cell), so build the four layers.
            let n = |side: usize, layer: usize| -> Vec3 { std::array::from_fn(|q| g.ksym[side][q].slice(2, layer..layer + 1)) };
            let (lo1, hi1) = (reflect3(p, &n(0, 1)), reflect3(p, &n(1, 0)));
            let (lo2, hi2) = (reflect3(&hi1, &n(0, 0)), reflect3(&lo1, &n(1, 1)));
            let lo = rec3(|f| luma::concat(2, [&lo2[f], &lo1[f]]));
            let hi = rec3(|f| luma::concat(2, [&hi1[f], &hi2[f]]));
            w.slice_mut(2, ..2).slice(0, 2..ni + 2).slice(1, 2..nj + 2).assign(lo).expect(ok);
            w.slice_mut(2, nk + 2..).slice(0, 2..ni + 2).slice(1, 2..nj + 2).assign(hi).expect(ok);
        } else {
            let mirror_lo = rec3(|f| p[f].pad([(0, 0), (0, 0), (2, 0)], PadMode::Mirror).slice(2, 0..2));
            w.slice_mut(2, ..2).slice(0, 2..ni + 2).slice(1, 2..nj + 2).assign(reflect3(&mirror_lo, &g.ksym[0])).expect(ok);
            let mirror_hi = rec3(|f| p[f].pad([(0, 0), (0, 0), (0, 2)], PadMode::Mirror).slice(2, nk..nk + 2));
            w.slice_mut(2, nk + 2..).slice(0, 2..ni + 2).slice(1, 2..nj + 2).assign(reflect3(&mirror_hi, &g.ksym[1])).expect(ok);
        }
    }

    /// `res = dU/dt` of the conserved state `u` (`[ni, nj, nk]` fields) and, if asked, the local time step.
    ///
    /// ```text
    /// dU/dt = -(1/V) sum_faces S F.n        dt = cfl V / (1/2 sum_faces (|u.n| + c) S)        (zero in solid cells)
    /// ```
    pub fn eval_residual(&self, u: &Rec3, lim: Limiter, compute_dt: bool) -> (Rec3, Option<A>) {
        let mu_t = self.state_mu_t();
        let r = self.residual3(u, mu_t.as_ref(), lim, compute_dt);
        (r.res, r.dt)
    }

    /// Step 1 of `eval_residual`: the primitives `p = cons_to_prim3(u)` and the padded primitives `w` with the ghosts (the `Staging` cache applied).
    fn padded3(&self, u: &Rec3) -> (Rec3, Rec3) {
        let p = cons_to_prim3(u.clone());
        let mut w = p.pad([(2, 2), (2, 2), (2, 2)], PadMode::Edge);
        self.assign_ghosts3(&p, &mut w);
        let w = if self.staging == Staging::Fusion { w } else { w.cache() };
        (p, w)
    }

    /// The region of the cells `lo..hi` of every axis of a padded record, the stencil cell `off` (0..4: `m2, m1, p0, p1`) of the faces `lo..hi` along `dir`.
    fn stencil3(w: &Rec3, dir: usize, off: usize, lo: [usize; 3], hi: [usize; 3]) -> Rec3 {
        let mut v = w.clone();
        for a in 0..3 {
            v = if a == dir { v.slice(a, lo[a] + off..hi[a] + off) } else { v.slice(a, lo[a] + 2..hi[a] + 2) };
        }
        v
    }

    /// Steps 2 and 3 of `eval_residual`: the face fluxes of the padded primitives `w`, as `[fi, fj, fk]` in the face layouts of the grid. Face `f` of a
    /// line sees the cells `f - 2 .. f + 1`. The faces next to solid cells are recomputed from the wall stencils (`SolidFaces3`) and written over.
    fn face_fluxes3(&self, w: &Rec3, lim: Limiter) -> [Rec3; 3] {
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let g = &self.geo;
        let split = self.staging == Staging::Split;
        let cached = matches!(self.staging, Staging::Fluxes | Staging::Split);
        let n = [ni, nj, nk];
        let lm = self.cfg.low_mach;
        std::array::from_fn(|dir| {
            // the cells along each grid line with their ghosts, the other axes cut to the interior
            let mut line = w.clone();
            for a in (0..3).filter(|&a| a != dir) {
                line = line.slice(a, 2..n[a] + 2);
            }
            let cell = |off: usize| line.slice(dir, off..n[dir] + 1 + off);
            let fg = &g.f[dir];
            let mut f = if dir == 2 && self.k_wall_flux {
                // the two planes of a one-cell wedge: the pressure of the cell on the face normal, nothing else (no mass, no energy)
                let p = w[4].slice(0, 2..ni + 2).slice(1, 2..nj + 2).slice(2, 2..3);
                let pp = luma::concat(2, [&p, &p]);
                // two distinct expressions for the zero mass and energy fluxes (a region assign refuses records that share an array)
                let (z_mass, z_energy) = (c(0.0) * &pp, &pp * &fg.n[0] * c(0.0));
                [z_mass, &pp * &fg.n[0], &pp * &fg.n[1], &pp * &fg.n[2], z_energy]
            } else {
                face_flux_lm3(&cell(0), &cell(1), &cell(2), &cell(3), lim, &fg.n, &fg.rot, lm, split)
            };
            // `Fluxes` and up: evaluate the face fluxes before the solid faces are written over them (a lazy write would make every face pay for both
            // expressions); the residual and the stage update then fuse into the next stage's padded record
            if cached {
                f = f.cache();
            }
            if let Some(sf) = &g.solid_faces[dir] {
                self.solid_fluxes3(dir, sf, w, lim, &mut f);
            }
            f
        })
    }

    /// The fluxes of the faces of the box `sf` (see [`SolidFaces3`]), written into the flux record `f` of direction `dir`.
    fn solid_fluxes3(&self, dir: usize, sf: &SolidFaces3, w: &Rec3, lim: Limiter, f: &mut Rec3) {
        let g = &self.geo;
        let (lo, hi) = (sf.lo, sf.hi);
        let split = self.staging == Staging::Split;
        let region = |a: &A| {
            let mut v = a.clone();
            for ax in 0..3 {
                v = v.slice(ax, lo[ax]..hi[ax]);
            }
            v
        };
        let fg = &g.f[dir];
        let n: Vec3 = std::array::from_fn(|q| region(&fg.n[q]));
        let rot: [A; 2] = std::array::from_fn(|q| region(&fg.rot[q]));
        // the normals of the previous and the next face of the line (`pad` repeats the edge face: those are used only where the neighbour exists)
        let widths = |a: &A| match dir {
            0 => a.pad([(1, 1)], PadMode::Edge),
            1 => a.pad([(0, 0), (1, 1)], PadMode::Edge),
            _ => a.pad([(0, 0), (0, 0), (1, 1)], PadMode::Edge),
        };
        let shifted = |q: usize, s: usize| {
            let mut v = widths(&fg.n[q]);
            for ax in 0..3 {
                v = if ax == dir { v.slice(ax, lo[ax] + s..hi[ax] + s) } else { v.slice(ax, lo[ax]..hi[ax]) };
            }
            v
        };
        let n_prev: Vec3 = std::array::from_fn(|q| shifted(q, 0));
        let n_next: Vec3 = std::array::from_fn(|q| shifted(q, 2));
        let (wm2, wm1, wp0, wp1) =
            (Self::stencil3(w, dir, 0, lo, hi), Self::stencil3(w, dir, 1, lo, hi), Self::stencil3(w, dir, 2, lo, hi), Self::stencil3(w, dir, 3, lo, hi));
        let (r_p0, r_p1) = (self.wall_ghost3(&wp0, &n), self.wall_ghost3(&wp1, &n));
        let (r_m1, r_m2) = (self.wall_ghost3(&wm1, &n), self.wall_ghost3(&wm2, &n));
        let m1 = select3(&sf.ma, &r_p0, &wm1);
        let m2 = select3(&sf.ma, &select3(&sf.mc, &r_p0, &r_p1), &select3(&sf.m2s, &self.wall_ghost3(&wm1, &n_prev), &wm2));
        let p0 = select3(&sf.mb, &r_m1, &wp0);
        let p1 = select3(&sf.mb, &select3(&sf.md, &r_m1, &r_m2), &select3(&sf.m1s, &self.wall_ghost3(&wp0, &n_next), &wp1));
        // the exit face of a motor jet: the stencil behind it is the jet state, `(jet, jet, p0, p1)` (the CPU `Family::jet_at`); the same two cells on the other side of a
        // face whose upper cell is a jet cell
        let (m1, m2, p0, p1) = match &self.jet {
            Some(j) => {
                let jp: Rec3 = rec3(|q| &wm1[q] * c(0.0) + c(j.state.state[q]));
                (select3(&sf.ja, &jp, &m1), select3(&sf.ja, &jp, &m2), select3(&sf.jb, &jp, &p0), select3(&sf.jb, &jp, &p1))
            }
            None => (m1, m2, p0, p1),
        };
        let flux = face_flux_lm3(&m2, &m1, &p0, &p1, lim, &n, &rot, self.cfg.low_mach, split);
        let flux = rec3(|k| select(&sf.z, c(0.0), &flux[k]));
        let mut dest = f.slice_mut(0, lo[0]..hi[0]).slice(1, lo[1]..hi[1]).slice(2, lo[2]..hi[2]);
        dest.assign(flux).expect("solid face flux assign: shapes fixed by the grid");
    }

    /// Steps 4 and 5 of `eval_residual`: the cell residual and the time step from the face fluxes `f = [fi, fj, fk]` and the primitives `p`; both are
    /// zero in solid cells (selects, so nothing computed inside the solid reaches the state or the residual norm).
    fn cell_residual3(&self, p: &Rec3, f: &[Rec3; 3], compute_dt: bool, diff: Option<&[A; 3]>) -> (Rec3, Option<A>) {
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let g = &self.geo;
        let (ai, aj, ak) = (&g.f[0].area, &g.f[1].area, &g.f[2].area);
        let (air, ail) = (ai.slice(0, 1..ni + 1), ai.slice(0, 0..ni));
        let (ajr, ajl) = (aj.slice(1, 1..nj + 1), aj.slice(1, 0..nj));
        let (akr, akl) = (ak.slice(2, 1..nk + 1), ak.slice(2, 0..nk));

        // 4. cell residual
        let res = rec3(|k| {
            let (fir, fil) = (f[0][k].slice(0, 1..ni + 1), f[0][k].slice(0, 0..ni));
            let (fjr, fjl) = (f[1][k].slice(1, 1..nj + 1), f[1][k].slice(1, 0..nj));
            let (fkr, fkl) = (f[2][k].slice(2, 1..nk + 1), f[2][k].slice(2, 0..nk));
            let r = -((&fir * &air - &fil * &ail) + (&fjr * &ajr - &fjl * &ajl) + (&fkr * &akr - &fkl * &akl)) * &g.inv_v;
            match &g.fluid {
                Some(fluid) => select(fluid, r, c(0.0)),
                None => r,
            }
        });

        // 5. local time step
        let dt = compute_dt.then(|| {
            let cs = (c(GAMMA) * &p[4] / &p[0]).sqrt();
            let lam = |fg: &FaceGeo3, ax: usize, r: usize| {
                let side = |a: &A| a.slice(ax, r..r + [ni, nj, nk][ax]);
                let un = &p[1] * side(&fg.n[0]) + &p[2] * side(&fg.n[1]) + &p[3] * side(&fg.n[2]);
                (un.abs() + &cs) * side(&fg.area)
            };
            let mut sum = lam(&g.f[0], 0, 0) + lam(&g.f[0], 0, 1) + lam(&g.f[1], 1, 0) + lam(&g.f[1], 1, 1);
            if !self.no_dt_k {
                sum = sum + lam(&g.f[2], 2, 0) + lam(&g.f[2], 2, 1);
            }
            // the viscous run adds the diffusive rates `2 lv A^2 / V` of the directions (`compute_dt`: `1 / dt = (sum L + sum d) / (cfl V)`)
            let mut den = c(0.5) * &sum;
            if let Some(d) = diff {
                den = den + &d[0] + &d[1];
                if !self.no_dt_k {
                    den += &d[2];
                }
            }
            let dt = c(self.cfg.cfl) * &g.vol / den;
            // the axis cells of a wedge: at most `WEDGE_AXIS_DT r_c / c` (`Solver3::compute_dt`)
            let dt = match &self.axis_dt {
                Some(a) => select(&a.mask, dt.minimum(&a.cap / &cs), &dt),
                None => dt,
            };
            match &g.fluid {
                Some(fluid) => select(fluid, dt, c(0.0)),
                None => dt,
            }
        });
        (res, dt)
    }

    /// Fluid cells take `new`, solid cells keep `old` (they are never updated).
    fn keep_solid3(&self, new: Rec3, old: &Rec3) -> Rec3 {
        match &self.geo.fluid {
            Some(fluid) => rec3(|k| select(fluid, &new[k], &old[k])),
            None => new,
        }
    }

    /// One SSP-RK3 step with local time stepping; returns the new state and the density residual of the first stage
    /// `sqrt(sum (d rho / dt)^2 / N)`.
    ///
    /// ```text
    /// u1 = u0 + dt L(u0),   u2 = 3/4 u0 + 1/4 (u1 + dt L(u1)),   u3 = 1/3 u0 + 2/3 (u2 + dt L(u2))
    /// ```
    fn rk3_3(&self, u0: &Rec3, mu_t: Option<&A>, lim: Limiter) -> (Rec3, A, A, Resid3) {
        let r0 = self.residual3(u0, mu_t, lim, true);
        let dt = r0.dt.clone().expect("the residual stage was called with compute_dt = true");
        let rn = self.residual_norm3(&r0.res[0]);
        let u1 = self.keep_solid3(rec3(|k| &u0[k] + &dt * &r0.res[k]), u0);
        let r1 = self.residual3(&u1, mu_t, lim, false);
        let u2 = self.keep_solid3(rec3(|k| c(0.25) * (&u1[k] + &dt * &r1.res[k]) + c(0.75) * &u0[k]), u0);
        let r2 = self.residual3(&u2, mu_t, lim, false);
        let u3 = self.keep_solid3(rec3(|k| c(2.0 / 3.0) * (&u2[k] + &dt * &r2.res[k]) + c(1.0 / 3.0) * &u0[k]), u0);
        // the turbulence update reads the last evaluation (`Solver3::turbulence_update` after the stages) and the time step of the first
        (u3, rn, dt, r2)
    }

    fn limiter3(&self, iter: usize) -> Limiter {
        if iter < self.cfg.first_order_iters {
            Limiter::FirstOrder
        } else {
            self.cfg.limiter
        }
    }

    /// Number of iterations from `self.iter` that use the same limiter (the traced step is specific to the limiter).
    fn phase_len3(&self, left: usize) -> usize {
        if self.iter < self.cfg.first_order_iters {
            left.min(self.cfg.first_order_iters - self.iter)
        } else {
            left
        }
    }

    /// One iteration of the configured scheme from the loop state.
    fn step3(&self, s: &State3, lim: Limiter) -> State3 {
        match self.cfg.scheme {
            Scheme3::Implicit(p) => self.implicit_step3(s, &p, lim),
            Scheme3::Explicit => {
                let mu_t = self.turb.as_ref().map(|_| s[9].clone());
                let (u, rn, dt, last) = self.rk3_3(&unpack3(s), mu_t.as_ref(), lim);
                let turb = self.turb_step3(s, &dt, &last);
                pack3(u, rn, &s[6] * c(1.0), turb, &s[10] * c(1.0))
            }
        }
    }

    /// `n` iterations as one `iterate_until_all` loop per limiter phase (the step is traced once and its kernels replayed, the density residual
    /// recorded on the GPU every iteration, the host waits once per `sync` iterations); returns the residual history.
    pub fn run(&mut self, n: usize, sync: usize) -> Result<Vec<f64>> {
        Ok(self.run_monitored(n, sync)?.0)
    }

    /// [`LazySolver3::run`] that also returns the force monitor of every iteration (`[Fx, Fz, Mz]` summed over the monitored classes, see
    /// [`LazySolver3::set_force_monitor`]; empty without a monitor): entry `i` belongs to the state the step `i` started from, one iteration behind the residual.
    pub fn run_monitored(&mut self, n: usize, sync: usize) -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
        let mut history = Vec::with_capacity(n);
        let mut forces = Vec::new();
        let monitor = self.monitor.is_some();
        let mut left = n;
        while left > 0 {
            let seg = self.phase_len3(left);
            let lim = self.limiter3(self.iter);
            let key = (lim, self.staging, [self.skip_k, self.no_dt_k, self.k_wall_flux, self.k_src], sync);
            // the loop is traced once per (limiter, configuration): a chunk then costs only its launches (`luma::compile_until_all`; the setters that change what the step
            // reads drop the compiled loop). A step Luma cannot compile (a read-back inside it) runs through `iterate_until_all`, traced again every call
            if self.compiled.as_ref().is_none_or(|c| c.key != key || c.max < seg) {
                self.compiled = None;
                let init = self.state.as_ref().ok_or_else(no_state)?;
                match luma::compile_until_all(init, records3(luma::Until::new(seg).check_every(sync), monitor), |s, _| self.step3(s, lim)) {
                    Ok(run) => self.compiled = Some(CompiledLoop3 { key, max: seg, run }),
                    Err(luma::LumaError::Unsupported(_)) => {}
                    Err(e) => return Err(e),
                }
            }
            let init = self.state.take().ok_or_else(no_state)?;
            let run = match &mut self.compiled {
                Some(c) => c.run.run_steps(init, seg)?,
                None => luma::iterate_until_all(init, records3(luma::Until::new(seg).check_every(sync), monitor), |s, _| self.step3(s, lim))?,
            };
            history.extend(read_history3(&run)?);
            if monitor {
                forces.extend(read_forces3(&run)?);
            }
            self.state = Some(run.state);
            self.iter += seg;
            left -= seg;
        }
        Ok((history, forces))
    }

    /// `n` iterations as one `iterate_all` replay per limiter phase (no residual history).
    pub fn run_replay(&mut self, n: usize) -> Result<()> {
        let mut left = n;
        while left > 0 {
            let seg = self.phase_len3(left);
            let lim = self.limiter3(self.iter);
            let init = self.state.take().ok_or_else(no_state)?;
            self.state = Some(luma::iterate_all(init.each_ref(), seg, |s| self.step3(s, lim)));
            self.iter += seg;
            left -= seg;
        }
        Ok(())
    }

    /// `n` iterations calling the step every iteration and evaluating the new state with `eval_all` (no replay). Returns the evaluated residual
    /// scalars (read them with [`Residuals::read`]).
    pub fn run_eval(&mut self, n: usize) -> Result<Residuals> {
        let mut history = Vec::with_capacity(n);
        let mut s = self.state.take().ok_or_else(no_state)?;
        for _ in 0..n {
            s = self.step3(&s, self.limiter3(self.iter));
            luma::eval_all(&s.iter().collect::<Vec<_>>())?;
            history.push(s[5].clone());
            self.iter += 1;
        }
        self.state = Some(s);
        Ok(Residuals(history))
    }

    /// Waits for the GPU.
    pub fn sync(&self) -> Result<()> {
        self.ctx.sync()
    }

    /// The residual `dU/dt` and time step of the current state, on the host (`ni * nj * nk` records, `dt`).
    pub fn residual_f64(&self, lim: Limiter) -> Result<(Vec<Cons3>, Vec<f64>)> {
        let u = unpack3(self.state.as_ref().ok_or_else(no_state)?);
        let (r, dt) = self.eval_residual(&u, lim, true);
        let r: Vec<Vec<f64>> = r.iter().map(to_f64).collect::<Result<_>>()?;
        let dt = to_f64(&dt.ok_or_else(|| luma::LumaError::Unsupported("residual stage returned no time step".into()))?)?;
        Ok(((0..self.ni * self.nj * self.nk).map(|q| [r[0][q], r[1][q], r[2][q], r[3][q], r[4][q]]).collect(), dt))
    }

    /// GPU time per kernel of one full SSP-RK3 step (three residual evaluations and the stage updates) as the `eval` loop launches it, and where
    /// fusion cut the expressions.
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

    /// GPU time per kernel of `n` iterations of the `iterate_all` replay (the profile adds up all `n`), after a warm-up call that traces and
    /// compiles it.
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

    /// Host seconds to build the lazy expression of one SSP-RK3 step from the current state, `n` times, without evaluating anything.
    pub fn trace_seconds(&self, n: usize) -> f64 {
        let u = unpack3(self.state.as_ref().expect("trace_seconds needs a state: GpuSolver3::from_host sets it"));
        let lim = self.cfg.limiter;
        let t0 = std::time::Instant::now();
        for _ in 0..n {
            std::hint::black_box(self.rk3_3(&u, None, lim));
        }
        t0.elapsed().as_secs_f64()
    }

    /// Host seconds to build the lazy expression of one step of the configured scheme (the implicit step when `Scheme3::Implicit`) from the current state, `n` times, without
    /// evaluating anything: what the loop's one trace costs, and what an eval-per-iteration loop pays every iteration.
    pub fn trace_step_seconds(&self, n: usize) -> f64 {
        let s = self.state.as_ref().expect("trace_step_seconds needs a state");
        let lim = self.limiter3(self.iter);
        let t0 = std::time::Instant::now();
        for _ in 0..n {
            std::hint::black_box(self.step3(s, lim));
        }
        t0.elapsed().as_secs_f64()
    }
}
