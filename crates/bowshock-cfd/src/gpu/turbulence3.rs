// The 3D SST turbulence update in array ops (stage S5a): the port of `crate::turbulence3` (read its module header: it is the specification). `include!`d twice
// by `gpu/mod.rs` after `viscous3.rs` (it reuses `S`, `c`, `up`, `to_f64`, `lhs`, `from_lhs`, `A`, `Rec3`, `Vec3`, `rec3`, `VGeo3`, `VCell3`, `Grad12`, `pad_edge3`,
// `LazySolver3::{face_mu3, staged3}` of the files before it; every name of this file carries a `3`).
//
// Every function mirrors the CPU routine of the same name in `crate::turbulence3`, expression by expression:
//
//   wall_state_pass      -> `wall_state_pass3`     Reichardt-law inversion (`WALL_ITERATIONS` fixed Newton steps), Aupoix rough-wall state (`AUPOIX_ITERATIONS` fixed-point
//                                                  steps) for `ks > 0`, one state `[k_w, omega_w, mu_t,w]` per cell with a no-slip face
//   scalar_gradients     -> `scalar_gradients3`    Green-Gauss gradient of the `N` scalars (ghost sides: free stream, wall ghost `2 phi_w - phi`, the cell itself)
//   blends               -> `blends3`
//   face_data            -> `face_data3`           mass flow, conductance, diffusive flux, the two sides (`TFace3`), reflected scalar gradient across a symmetry plane
//   cell_terms           -> `cell_terms3`          `R = rho dphi/dt`, `diag0`, the line coefficients
//   solve_and_update     -> `solve_and_update3`    three scalar tridiagonal line solves (no k line on a wedge: `skip_k`), clipped update, new eddy viscosity, `rnorm`
//
// The model is a trait `TurbModel3<N>` of `N` scalar equations like the CPU `TurbModel<N>`; `Sst3` is the `N = 2` SST-2003. Everything is a plain function of its inputs
// (`TGeo3`: the host-built masks, `VGeo3`: the viscous face data, `TDev3`: the per-cell device inputs); the wall distance is computed on the host (`walldist3`).
//
// Where the array code cannot be the CPU code (the f32 round-off aside):
//
// * a cell that is solid on the CPU is skipped; here every kernel runs on every cell and the solid ones are replaced by the CPU's value of such a cell (zero, `d0 = 1`) with a
//   select at the end of the stage; solid cells of the inputs must be finite (the wall distance of a solid cell is `inf` on the host: the probe clamps it);
// * `u_tau = 0` at rest: the CPU returns early, here the Newton iteration of a cell at rest is a 0/0 whose result is replaced by 0;
// * `hypot` is `m sqrt(1 + (n / m)^2)` (Luma has no `hypot` for `Df32`, and the square overflows in f32 for large `m`), `a^4` of the blending function is bounded by `1e3`
//   (`tanh` is 1 there; `a^4` would overflow f32), as in the 2D `turbulence.rs`;
// * the line solves are Luma's `solve_tridiagonal_parts` in f32 (the left-hand side is f32 in both precisions, as everywhere in the implicit schemes).

use crate::turbulence as sst2;
use crate::turbulence3 as cpu3;
use crate::viscous3 as cv3;

/// What a model needs of every cell, as arrays of `[ni, nj, nk]` (the CPU `CellIn`).
pub struct CellIn3<const N: usize> {
    pub rho: A,
    pub nu: A,
    /// Distance of the centroid to the nearest wall.
    pub d: A,
    pub phi: [A; N],
    /// Gradient `[d_x, d_y, d_z]` of each of the `N` scalars.
    pub grad: [[A; 3]; N],
    /// `S^2` of the mean flow.
    pub s2: A,
    /// Current eddy viscosity.
    pub mu_t: A,
}

/// Blending function `F1` and the cross diffusion `X` of every cell.
pub struct Blend3 {
    pub f1: A,
    pub x: A,
}

/// A turbulence model as a list of `N` scalar transport equations (the CPU `TurbModel<N>` on arrays): free-stream values, the diffusion coefficients, sources and
/// linearised destruction, the wall face values, the clipping and the eddy viscosity.
pub trait TurbModel3<const N: usize> {
    /// Free-stream values (`Inf` ghosts, clipping floors).
    fn inf(&self) -> [f64; N];
    /// `[sigma_1, sigma_2]` of equation `e`.
    fn sigmas(&self, e: usize) -> [f64; 2];
    /// `F1` and `X` of every cell.
    fn blend(&self, cell: &CellIn3<N>) -> Blend3;
    /// Sources `S_e` and the linearised destruction coefficients `D_e`.
    fn terms(&self, cell: &CellIn3<N>, b: &Blend3) -> ([A; N], [A; N]);
    /// The face value of equation `e` at a no-slip wall from the wall state `[k_w, omega_w, mu_t,w]` (`None`: zero gradient).
    fn wall_value(&self, e: usize, wall: &[A; 3]) -> Option<A>;
    /// Clip of the updated value of equation `e`.
    fn clip(&self, e: usize, old: &A, new: &A) -> A;
    /// The eddy viscosity from the (new) scalars of every cell.
    fn eddy_viscosity(&self, cell: &CellIn3<N>) -> A;
}

/// The Menter SST-2003 model `[k, omega]` (the CPU `Sst`).
#[derive(Clone, Copy, Debug)]
pub struct Sst3 {
    /// Reynolds number `rho_inf c_inf d / mu_inf` (`omega_inf = 1e-6 re_c`, the eddy-viscosity cap `1e5 / re_c`).
    pub re_c: f64,
}

impl Sst3 {
    /// `(sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))`.
    fn args(nu: &A, k: &A, om: &A, d: &A) -> (A, A) {
        (k.sqrt() / (c(sst2::BETA_STAR) * om * d), c(500.0) * nu / (d * d * om))
    }
}

impl TurbModel3<2> for Sst3 {
    fn inf(&self) -> [f64; 2] {
        [sst2::K_INF, sst2::omega_inf(self.re_c)]
    }

    fn sigmas(&self, e: usize) -> [f64; 2] {
        if e == 0 {
            [sst2::SIGMA_K1, sst2::SIGMA_K2]
        } else {
            [sst2::SIGMA_W1, sst2::SIGMA_W2]
        }
    }

    fn blend(&self, cell: &CellIn3<2>) -> Blend3 {
        let [k, om] = &cell.phi;
        let g = &cell.grad;
        let x = c(2.0) * &cell.rho * c(sst2::SIGMA_W2) * (&g[0][0] * &g[1][0] + &g[0][1] * &g[1][1] + &g[0][2] * &g[1][2]) / om;
        let cd = x.maximum(c(1e-20));
        let (t1, t2) = Sst3::args(&cell.nu, k, om, &cell.d);
        let t3 = c(4.0) * &cell.rho * c(sst2::SIGMA_W2) * k / (&cd * &cell.d * &cell.d);
        // (`a` is bounded by `1e3` only to keep `a^4` finite in f32: `tanh` is 1 there)
        let a = t1.maximum(t2).minimum(t3).minimum(c(1e3));
        let a2 = &a * &a;
        Blend3 { f1: (&a2 * &a2).tanh(), x }
    }

    fn terms(&self, cell: &CellIn3<2>, b: &Blend3) -> ([A; 2], [A; 2]) {
        let [k, om] = &cell.phi;
        let rho = &cell.rho;
        let prod = (&cell.mu_t * &cell.s2).minimum(c(10.0 * sst2::BETA_STAR) * rho * k * om);
        let beta = &b.f1 * c(sst2::BETA1) + (c(1.0) - &b.f1) * c(sst2::BETA2);
        let gamma = &b.f1 * c(sst2::GAMMA1) + (c(1.0) - &b.f1) * c(sst2::GAMMA2);
        (
            [
                &prod - c(sst2::BETA_STAR) * rho * om * k,
                &gamma * rho * &prod / cell.mu_t.maximum(c(1e-30)) - &beta * rho * om * om + (c(1.0) - &b.f1) * &b.x,
            ],
            [c(sst2::BETA_STAR) * om, c(2.0) * &beta * om],
        )
    }

    fn wall_value(&self, e: usize, wall: &[A; 3]) -> Option<A> {
        Some(wall[e].clone())
    }

    fn clip(&self, e: usize, old: &A, new: &A) -> A {
        let inf = self.inf();
        new.maximum(c(0.1) * old).maximum(c(1e-3 * inf[e]))
    }

    fn eddy_viscosity(&self, cell: &CellIn3<2>) -> A {
        let [k, om] = &cell.phi;
        let (t1, t2) = Sst3::args(&cell.nu, k, om, &cell.d);
        // (`b` is bounded like the `a` of `blend`)
        let b = (c(2.0) * t1).maximum(t2).minimum(c(1e3));
        let f2 = (&b * &b).tanh();
        let s = cell.s2.maximum(c(0.0)).sqrt();
        (&cell.rho * c(sst2::A1) * k / (c(sst2::A1) * om).maximum(s * f2)).minimum(c(1e5 / self.re_c))
    }
}

/// `S^2 = 2 S_ij S_ij - 2/3 div^2` of the gradient record of the cell pass (`turbulence3::strain2`).
fn strain2_3(g: &[A]) -> A {
    let div = &g[0] + &g[4] + &g[8];
    let mut s: Option<A> = None;
    for a in 0..3 {
        for b in 0..3 {
            let sab = c(0.5) * (&g[3 * a + b] + &g[3 * b + a]);
            let term = &sab * &sab;
            s = Some(match s {
                None => term,
                Some(x) => x + term,
            });
        }
    }
    c(2.0) * s.expect("nine terms") - c(2.0 / 3.0) * &div * &div
}

/// `sqrt(a^2 + b^2)` of two positive fields without the overflow of the squares.
fn hypot3(a: &A, b: &A) -> A {
    let (m, n) = (a.maximum(b), a.minimum(b));
    let r = &n / &m;
    &m * (c(1.0) + &r * &r).sqrt()
}

/// One Newton step `u <- max(u - (u U+(y+) - U) / (U+ + y+ dU+/dy+), u / 2)` of the inversion of Reichardt's law of the wall (`turbulence3::u_tau_law`).
fn newton_step3(u: &A, nu: &A, speed: &A, d: &A) -> A {
    let kappa = cpu3::KAPPA;
    let yp = d * u / nu;
    let (e11, e3) = ((-&yp / c(11.0)).exp(), (-&yp / c(3.0)).exp());
    let up = (c(1.0) + c(kappa) * &yp).ln() / c(kappa) + c(7.8) * (c(1.0) - &e11 - &yp / c(11.0) * &e3);
    let dup = c(1.0) / (c(1.0) + c(kappa) * &yp) + c(7.8) * (&e11 / c(11.0) - &e3 / c(11.0) + &yp / c(33.0) * &e3);
    (u - (u * &up - speed) / (&up + &yp * &dup)).maximum(c(0.5) * u)
}

/// The per-grid masks of the turbulence kernels of one face family, in the face layout `[n0, n1, n2]` of `dir` (host-built from `face_cells`, the solid mask and the
/// scalar boundary kinds).
struct TFam3 {
    /// The lower / upper side is a domain ghost with the free-stream scalars (`ScalarBc::Inf`).
    inf: [Array<bool>; 2],
    /// The lower / upper side is a domain ghost across a symmetry plane (`SymSides`: the scalar gradient of that side is reflected).
    sym: [Array<bool>; 2],
    /// The lower / upper side is a fluid cell: the face couples to its neighbour (a ghost, a wall and a face without flux do not).
    coupled: [Array<bool>; 2],
    /// The lower / upper side is a domain ghost that follows its cell (`ScalarBc::Cell`): its face adds nothing to the diagonal.
    follows: [Array<bool>; 2],
}

/// Flat index of face `f = [i, j, k]` in the layout of the family `dir` (`Grid3::{iface, jface, kface}`).
fn face_q3(grid: &Grid3, dir: usize, f: [usize; 3]) -> usize {
    let [n0, n1, _] = cv3::face_shape(grid, dir);
    (f[2] * n1 + f[1]) * n0 + f[0]
}

/// The host-built geometry of the turbulence kernels.
pub(crate) struct TGeo3 {
    dir: [TFam3; 3],
    /// Fluid cells.
    fluid: Array<bool>,
    /// The flat indices of the fluid cells with a no-slip face (`None` without any): the wall state is computed on these cells only.
    wall_idx: Option<Array<u32>>,
    /// `1 / V` (zero in solid cells) and `V` (one in solid cells).
    inv_v: A,
    vol: A,
    n_active: usize,
}

impl TGeo3 {
    /// `solid` is the per-cell update mask, `sym` the symmetry sides of the domain box, `bc` the scalar ghost kind of the six domain sides, `vg` the viscous geometry of the
    /// same grid, mask and `sym` (its no-slip masks give the wall cells).
    pub(crate) fn new(ctx: &Context, grid: &Grid3, solid: &[bool], sym: &cv3::SymSides, bc: &cpu3::ScalarSides, vg: &VGeo3) -> Result<TGeo3> {
        let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
        let n = [ni, nj, nk];
        let family = |dir: usize| -> Result<TFam3> {
            let shape = cv3::face_shape(grid, dir);
            let nf = shape[0] * shape[1] * shape[2];
            let (mut inf, mut sm, mut coupled, mut follows) = ([vec![false; nf], vec![false; nf]], [vec![false; nf], vec![false; nf]], [vec![false; nf], vec![false; nf]], [vec![false; nf], vec![false; nf]]);
            for k in 0..shape[2] {
                for j in 0..shape[1] {
                    for i in 0..shape[0] {
                        let q = (k * shape[1] + j) * shape[0] + i;
                        let idx = [i, j, k][dir];
                        sm[0][q] = idx == 0 && sym[2 * dir];
                        sm[1][q] = idx == n[dir] && sym[2 * dir + 1];
                        let Some(fc) = cv3::face_cells(grid, solid, dir, i, j, k) else { continue };
                        for s in 0..2 {
                            coupled[s][q] = fc.kind[s] == cv3::SideKind::Cell;
                            let domain = fc.kind[s] == cv3::SideKind::Domain;
                            inf[s][q] = domain && bc[2 * dir + s] == cpu3::ScalarBc::Inf;
                            follows[s][q] = domain && bc[2 * dir + s] == cpu3::ScalarBc::Cell;
                        }
                    }
                }
            }
            let mask = |v: &[Vec<bool>; 2]| -> Result<[Array<bool>; 2]> { Ok([Array::from_slice_on(ctx, &v[0], shape)?, Array::from_slice_on(ctx, &v[1], shape)?]) };
            Ok(TFam3 { inf: mask(&inf)?, sym: mask(&sm)?, coupled: mask(&coupled)?, follows: mask(&follows)? })
        };
        let dir = [family(0)?, family(1)?, family(2)?];
        let shape = [ni, nj, nk];
        let fluid = Array::from_slice_on(ctx, &solid.iter().map(|s| !s).collect::<Vec<_>>(), shape)?;
        // a fluid cell with a no-slip face among its six (`wall_state_pass`)
        let noslip: [Vec<bool>; 3] = [vg.dir[0].noslip.to_vec()?, vg.dir[1].noslip.to_vec()?, vg.dir[2].noslip.to_vec()?];
        let mut wall: Vec<u32> = Vec::new();
        for c in 0..ni * nj * nk {
            if solid[c] {
                continue;
            }
            let (i, j, k) = (c % ni, (c / ni) % nj, c / (ni * nj));
            let has = (0..3).any(|d| {
                let lo = [i, j, k];
                let mut hi = lo;
                hi[d] += 1;
                let q = |f: [usize; 3]| face_q3(grid, d, f);
                noslip[d][q(lo)] || noslip[d][q(hi)]
            });
            if has {
                wall.push(c as u32);
            }
        }
        let wall_idx = if wall.is_empty() { None } else { Some(Array::from_slice_on(ctx, &wall, [wall.len()])?) };
        Ok(TGeo3 {
            dir,
            fluid,
            wall_idx,
            inv_v: up(ctx, &grid.vol.iter().zip(solid).map(|(v, s)| if *s { 0.0 } else { 1.0 / v }).collect::<Vec<_>>(), shape)?,
            vol: up(ctx, &grid.vol.iter().zip(solid).map(|(v, s)| if *s { 1.0 } else { *v }).collect::<Vec<_>>(), shape)?,
            n_active: solid.iter().filter(|s| !**s).count().max(1),
        })
    }
}

/// The per-cell device inputs of the update (the arrays of the CPU `TurbIn`, `[ni, nj, nk]` unless stated).
pub(crate) struct TDev3<const N: usize> {
    /// Density.
    rho: A,
    /// `mu` and the mean-flow gradient of the viscous cell pass.
    cell: VCell3,
    /// Eddy viscosity of the previous update.
    mu_t: A,
    /// `[k_w, omega_w, mu_t,w]` of the wall cells (read at cells with a no-slip face only).
    wall: [A; 3],
    /// Wall distance of the cell centroids (finite).
    dwall: A,
    /// `F[0]` of the inviscid face flux of the three families (face layouts).
    mass: [A; 3],
    /// Pseudo time step per cell.
    dt: A,
    /// The scalars.
    turb: [A; N],
}

/// The scalar convection and diffusion data of every face of one family (the CPU `TFace`): face layout of the family.
pub(crate) struct TFace3<const N: usize> {
    m: A,
    g: [A; N],
    flux: [A; N],
    l: [A; N],
    r: [A; N],
}

/// Residual and line coefficients of every cell (the CPU `CellTerms`), per scalar; `[direction][scalar]`.
pub(crate) struct CellTerms3<const N: usize> {
    r: [A; N],
    d0: [A; N],
    lo: [[A; N]; 3],
    dd: [[A; N]; 3],
    up: [[A; N]; 3],
}

/// The result of an update: the new scalars, the new eddy viscosity and the `L2` norm of the residual `R_e` of every equation over the fluid cells (one element each).
pub(crate) struct TUpdate3<const N: usize> {
    turb: [A; N],
    mu_t: A,
    rnorm: [A; N],
}

/// `N` scalar tridiagonal systems (`lo x_{m-1} + d x_m + up x_{m+1} = rhs`) along `dir` in one launch of Luma's solver, in f32 (the left-hand side).
fn tridiag3<const N: usize>(dir: usize, lo: &[A; N], d: &[A; N], up: &[A; N], rhs: &[A; N]) -> [A; N] {
    let to32 = |v: &[A; N]| -> [Array<f32>; N] { std::array::from_fn(|e| lhs(&v[e])) };
    let (lo, d, up, rhs) = (to32(lo), to32(d), to32(up), to32(rhs));
    let systems: [(&Array<f32>, &Array<f32>, &Array<f32>, &Array<f32>); N] = std::array::from_fn(|e| (&lo[e], &d[e], &up[e], &rhs[e]));
    let sol = luma::solve_tridiagonal_parts(dir, systems);
    std::array::from_fn(|e| from_lhs(&sol[e]))
}

impl LazySolver3 {
    // -----------------------------------------------------------------------------------------------------------------
    // wall treatment

    /// Smooth-wall state `[k_w, omega_w, mu_t,w]` of the automatic treatment (`turbulence3::wall_state_smooth`): the friction velocity is the root of Reichardt's law
    /// by `WALL_ITERATIONS` Newton steps from `u_vis` (each evaluated, `staging`), zero at rest.
    fn wall_smooth3(&self, rho: &A, nu: &A, speed: &A, d: &A) -> [A; 3] {
        let mut u = (nu * speed / d).sqrt();
        for _ in 0..cpu3::WALL_ITERATIONS {
            u = self.staged3(newton_step3(&u, nu, speed, d));
        }
        let moving = speed.gt(c(0.0));
        let u_tau = select(&moving, u, c(0.0));
        let yp = d * &u_tau / nu;
        let yp2 = &yp * &yp;
        let yp4 = &yp2 * &yp2;
        let yb = &yp / c(cpu3::Y_BOOST);
        let yb2 = &yb * &yb;
        let boost = c(1.0) + c(cpu3::OMEGA_BOOST - 1.0) / (c(1.0) + &yb2 * &yb2);
        let omega_vis = c(6.0) * nu / (c(sst2::BETA1) * d * d) * &boost;
        let omega_log = &u_tau / (c(sst2::BETA_STAR.sqrt() * cpu3::KAPPA) * d);
        let k_w = &u_tau * &u_tau / c(sst2::BETA_STAR.sqrt()) * &yp4 / (&yp4 + c(cpu3::Y_K.powi(4)));
        let nu_t = (&u_tau * &u_tau * d / speed - nu).maximum(c(0.0)).minimum(c(sst2::NU_T_WALL_MAX) * nu);
        [k_w, hypot3(&omega_vis, &omega_log), rho * select(&moving, nu_t, c(0.0))]
    }

    /// Aupoix (2015) rough-wall state `[k_w, omega_w, nu_t,w]` (`turbulence::aupoix_wall`, as the 2D `LazySolver::aupoix_wall`): `AUPOIX_ITERATIONS` fixed-point iterations
    /// of `u_tau^2 = (nu + nu_t,w) U / d`, each evaluated (`staging`). `ks` positive.
    fn aupoix3(&self, nu: &A, speed: &A, d: &A, ks: &A) -> [A; 3] {
        let smooth = c(60.0) * nu / (c(sst2::BETA1) * d * d);
        let at = |u2: &A| -> [A; 3] {
            let u_tau = u2.sqrt();
            let ks_plus = (&u_tau * ks / nu).maximum(c(1e-30));
            let t = (&ks_plus / c(125.0)).tanh();
            let log10 = (&ks_plus / c(30.0)).ln() * c(std::f64::consts::LOG10_E);
            let k_plus = ((log10 + c(1.0) - &t) * &t).tanh().maximum(c(0.0)) * c(1.0 / sst2::BETA_STAR.sqrt());
            let om_rough = c(300.0) * nu / (ks * ks) / (c(15.0 / 4.0) / &ks_plus).tanh() + c(191.0) * &u_tau / ks * (c(1.0) - (-&ks_plus / c(250.0)).exp());
            let (k_w, om_w) = (k_plus * u2, om_rough.minimum(&smooth));
            let nu_t = (&k_w / &om_w).minimum(c(sst2::NU_T_WALL_MAX) * nu);
            [k_w, om_w, nu_t]
        };
        let mut u2 = nu * speed / d;
        for _ in 0..sst2::AUPOIX_ITERATIONS {
            u2 = self.staged3((nu + &at(&u2)[2]) * speed / d);
        }
        at(&u2)
    }

    /// `turbulence3::wall_state_pass`: the wall state `[k_w, omega_w, mu_t,w]` of every fluid cell with a no-slip face (zero elsewhere) from the unpadded primitives `p`,
    /// the viscosity `mu`, the wall distance `dwall` and the roughness per cell `ks` (`None`: all smooth). The state is computed on the gathered wall cells only (the
    /// Newton and fixed-point iterations are transcendental: in `Df32` they cost as much as the rest of the update on all cells) and scattered into zeros.
    fn wall_state_pass3(&self, tg: &TGeo3, p: &Rec3, mu: &A, dwall: &A, ks: Option<&A>) -> Result<[A; 3]> {
        let shape = [self.ni, self.nj, self.nk];
        let mut out: [A; 3] = [A::zeros_on(&self.ctx, shape)?, A::zeros_on(&self.ctx, shape)?, A::zeros_on(&self.ctx, shape)?];
        let Some(idx) = &tg.wall_idx else { return Ok(out) };
        let at = |a: &A| a.take(idx);
        let (rho, mu, d) = (at(&p[0]), at(mu), at(dwall));
        let nu = &mu / &rho;
        let (u, v, w) = (at(&p[1]), at(&p[2]), at(&p[3]));
        let speed = (&u * &u + &v * &v + &w * &w).sqrt();
        let smooth = self.wall_smooth3(&rho, &nu, &speed, &d);
        let state = match ks {
            None => smooth,
            Some(ks) => {
                let ks = at(ks);
                let rough_cell = ks.gt(c(0.0));
                let ks_safe = select(&rough_cell, &ks, c(1.0));
                let [k_w, om_w, nu_t] = self.aupoix3(&nu, &speed, &d, &ks_safe);
                let pick = |r: A, s: &A| select(&rough_cell, r, s);
                [pick(k_w, &smooth[0]), pick(om_w, &smooth[1]), pick(&rho * nu_t, &smooth[2])]
            }
        };
        for (o, v) in out.iter_mut().zip(&state) {
            o.put(idx, v)?;
        }
        Ok(out)
    }

    // -----------------------------------------------------------------------------------------------------------------
    // the update

    /// The scalars on the two sides of every face of `dir` (`TurbIn::side_phi`): the cells, the free-stream values in an `Inf` domain ghost, the wall ghost
    /// `2 phi_w - phi` of a no-slip face, a ghost of kind `Cell` the value of its cell.
    fn phi_sides3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, vg: &VGeo3, model: &M, turb: &[A; N], wall: &[A; 3], dir: usize) -> ([A; N], [A; N]) {
        let (l, r) = vg.cell_sides3(turb, dir);
        let (wl, wr) = vg.cell_sides3(wall, dir);
        let (f, t) = (&vg.dir[dir], &tg.dir[dir]);
        let inf = model.inf();
        let side = |ph: &[A; N], wc: &[A; 3], s: usize| -> [A; N] {
            std::array::from_fn(|e| {
                let v = match model.wall_value(e, wc) {
                    Some(fv) => select(&f.wall[s], c(2.0) * &fv - &ph[e], &ph[e]),
                    None => ph[e].clone(),
                };
                select(&t.inf[s], c(inf[e]), v)
            })
        };
        (side(&l, &wl, 0), side(&r, &wr, 1))
    }

    /// Cell data of a model (`TurbIn::cell_in`) from the scalars `phi` and their gradients.
    fn cell_in3<const N: usize>(&self, dev: &TDev3<N>, phi: &[A; N], grad: &[[A; 3]; N]) -> CellIn3<N> {
        CellIn3 {
            rho: dev.rho.clone(),
            nu: &dev.cell.mu / &dev.rho,
            d: dev.dwall.clone(),
            phi: phi.clone(),
            grad: grad.clone(),
            s2: strain2_3(&dev.cell.grad),
            mu_t: dev.mu_t.clone(),
        }
    }

    /// Kernel 1 (`turbulence3::scalar_gradients`): the Green-Gauss gradient `(1 / V) sum_faces phi_f S_f`, `phi_f = (phi_c + phi_neighbour) / 2`, of every scalar of every
    /// fluid cell (zero in solid cells).
    pub(crate) fn scalar_gradients3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, vg: &VGeo3, model: &M, dev: &TDev3<N>) -> [[A; 3]; N] {
        let n = [self.ni, self.nj, self.nk];
        let g = &self.geo;
        let mut acc: Vec<Option<A>> = vec![None; 3 * N];
        for dir in 0..3 {
            let (f, fg) = (&vg.dir[dir], &g.f[dir]);
            let (l, r) = self.phi_sides3(tg, vg, model, &dev.turb, &dev.wall, dir);
            let na: Vec3 = std::array::from_fn(|a| &fg.n[a] * &fg.area);
            for e in 0..N {
                let (lo, hi) = (pad_edge3(&dev.turb[e], dir, 1, 0), pad_edge3(&dev.turb[e], dir, 0, 1));
                // the face value seen from the cell below (through its upper face) and from the cell above (through its lower face)
                let from_below = select(&f.fc, c(0.5) * (&lo + &r[e]), c(0.0));
                let from_above = select(&f.fc, c(0.5) * (&hi + &l[e]), c(0.0));
                for a in 0..3 {
                    let term = (&from_below * &na[a]).slice(dir, 1..n[dir] + 1) - (&from_above * &na[a]).slice(dir, 0..n[dir]);
                    let q = 3 * e + a;
                    acc[q] = Some(match acc[q].take() {
                        None => term,
                        Some(x) => x + term,
                    });
                }
            }
        }
        let out: [[A; 3]; N] = std::array::from_fn(|e| std::array::from_fn(|a| select(&tg.fluid, acc[3 * e + a].take().expect("six faces") * &tg.inv_v, c(0.0))));
        self.staged3(out)
    }

    /// Kernel 2 (`turbulence3::blends`): `F1` and the cross diffusion `X` of every fluid cell (zero in solid cells).
    pub(crate) fn blends3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, model: &M, dev: &TDev3<N>, grad: &[[A; 3]; N]) -> Blend3 {
        let b = model.blend(&self.cell_in3(dev, &dev.turb, grad));
        let (f1, x) = self.staged3((select(&tg.fluid, &b.f1, c(0.0)), select(&tg.fluid, &b.x, c(0.0))));
        Blend3 { f1, x }
    }

    /// Kernel 3 (`turbulence3::face_data`): the scalar face data of every face of the three families (zero where the face carries no flux).
    pub(crate) fn face_data3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, vg: &VGeo3, model: &M, dev: &TDev3<N>, grad: &[[A; 3]; N], blend: &Blend3) -> [TFace3<N>; 3] {
        let g = &self.geo;
        let wall_mut = &dev.wall[2];
        let scal: [A; 3] = [dev.cell.mu.clone(), dev.mu_t.clone(), wall_mut.clone()];
        std::array::from_fn(|dir| {
            let (f, fg, t) = (&vg.dir[dir], &g.f[dir], &tg.dir[dir]);
            let (l, r) = self.phi_sides3(tg, vg, model, &dev.turb, &dev.wall, dir);
            let m = &dev.mass[dir] * &fg.area;
            let (sl, sr) = vg.cell_sides3(&scal, dir);
            let (mu_f, mut_f) = self.face_mu3(vg, dir, &sl, &sr);
            let (f1l, f1r) = vg.cell_sides3(&blend.f1, dir);
            // the gradient of a scalar across a symmetry plane: `R g = g - 2 (g.n) n` on the ghost side
            let reflect = |gr: &[A; 3], mask: &Array<bool>| -> [A; 3] {
                let dn = &gr[0] * &fg.n[0] + &gr[1] * &fg.n[1] + &gr[2] * &fg.n[2];
                std::array::from_fn(|a| select(mask, &gr[a] - c(2.0) * &dn * &fg.n[a], &gr[a]))
            };
            let (mut g_flux, mut g_cond) = (Vec::with_capacity(N), Vec::with_capacity(N));
            for e in 0..N {
                let [s1, s2] = model.sigmas(e);
                let sigma = c(0.5) * (&f1l * c(s1) + (c(1.0) - &f1l) * c(s2) + &f1r * c(s1) + (c(1.0) - &f1r) * c(s2));
                let coef = &mu_f + sigma * &mut_f;
                let (gl, gr) = vg.cell_sides3(&grad[e], dir);
                let (gl, gr) = (reflect(&gl, &t.sym[0]), reflect(&gr, &t.sym[1]));
                let gb: [A; 3] = std::array::from_fn(|a| c(0.5) * (&gl[a] + &gr[a]));
                let corr = (&r[e] - &l[e]) * &f.inv_d - (&gb[0] * &f.e[0] + &gb[1] * &f.e[1] + &gb[2] * &f.e[2]);
                let gn = (&gb[0] + &corr * &f.e[0]) * &fg.n[0] + (&gb[1] + &corr * &f.e[1]) * &fg.n[1] + (&gb[2] + &corr * &f.e[2]) * &fg.n[2];
                g_flux.push(select(&f.flux, &coef * &gn * &fg.area, c(0.0)));
                g_cond.push(select(&f.flux, &coef * &fg.area * &f.inv_d, c(0.0)));
            }
            let zero = |x: &A| select(&f.flux, x, c(0.0));
            let out = TFace3 {
                m: zero(&m),
                g: std::array::from_fn(|e| g_cond[e].clone()),
                flux: std::array::from_fn(|e| g_flux[e].clone()),
                l: std::array::from_fn(|e| zero(&l[e])),
                r: std::array::from_fn(|e| zero(&r[e])),
            };
            let (m, gg, flux, l, r) = self.staged3((out.m, out.g, out.flux, out.l, out.r));
            TFace3 { m, g: gg, flux, l, r }
        })
    }

    /// Kernel 4 (`turbulence3::cell_terms`): the residual `R = rho dphi/dt`, `diag0` and the line coefficients of every fluid cell (solid cells: zero and `d0 = 1`).
    pub(crate) fn cell_terms3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, model: &M, dev: &TDev3<N>, grad: &[[A; 3]; N], blend: &Blend3, tf: &[TFace3<N>; 3]) -> CellTerms3<N> {
        let n = [self.ni, self.nj, self.nk];
        let (sources, dest) = model.terms(&self.cell_in3(dev, &dev.turb, grad), blend);
        let mut rate: Vec<Option<A>> = vec![None; N];
        let blank = || -> [[Option<A>; N]; 3] { std::array::from_fn(|_| std::array::from_fn(|_| None)) };
        let (mut lo, mut up, mut dd) = (blank(), blank(), blank());
        for dir in 0..3 {
            let (f, t) = (&tf[dir], &tg.dir[dir]);
            // a neighbour that is a ghost, a solid cell or across a face without flux is not coupled; a ghost that follows its cell adds nothing to the diagonal
            let lower = t.coupled[0].slice(dir, 0..n[dir]);
            let upper = t.coupled[1].slice(dir, 1..n[dir] + 1);
            let lower_id = t.follows[0].slice(dir, 0..n[dir]);
            let upper_id = t.follows[1].slice(dir, 1..n[dir] + 1);
            let a_lo = f.m.slice(dir, 0..n[dir]).maximum(c(0.0));
            let a_up = (-f.m.slice(dir, 1..n[dir] + 1)).maximum(c(0.0));
            for e in 0..N {
                let (g_lo, g_up) = (f.g[e].slice(dir, 0..n[dir]), f.g[e].slice(dir, 1..n[dir] + 1));
                let term = &a_lo * (f.l[e].slice(dir, 0..n[dir]) - &dev.turb[e]) + &a_up * (f.r[e].slice(dir, 1..n[dir] + 1) - &dev.turb[e]) + f.flux[e].slice(dir, 1..n[dir] + 1)
                    - f.flux[e].slice(dir, 0..n[dir]);
                rate[e] = Some(match rate[e].take() {
                    None => term,
                    Some(x) => x + term,
                });
                let (c_lo, c_up) = (&a_lo + &g_lo, &a_up + &g_up);
                lo[dir][e] = Some(select(&lower, -&c_lo, c(0.0)));
                up[dir][e] = Some(select(&upper, -&c_up, c(0.0)));
                dd[dir][e] = Some(select(&lower_id, c(0.0), &c_lo) + select(&upper_id, c(0.0), &c_up));
            }
        }
        let rho_vol = &dev.rho * &tg.vol;
        let solid_to = |x: A, other: f64| select(&tg.fluid, x, c(other));
        let take = |o: &mut [[Option<A>; N]; 3]| -> [[A; N]; 3] { std::array::from_fn(|dir| std::array::from_fn(|e| solid_to(o[dir][e].take().expect("every coefficient is set"), 0.0))) };
        let r: [A; N] = std::array::from_fn(|e| solid_to(rate[e].take().expect("three directions") * &tg.inv_v + &sources[e], 0.0));
        let d0: [A; N] = std::array::from_fn(|e| solid_to(&rho_vol / &dev.dt + &rho_vol * &dest[e], 1.0));
        let (lo, dd, upc) = (take(&mut lo), take(&mut dd), take(&mut up));
        // evaluated in four groups (the residual and `diag0`, then the coefficients of each direction): the residual feeds the line solves and the norm, the coefficients
        // the solves, and a `Df32` cell term is expensive to evaluate twice
        let (r, d0) = self.staged3((r, d0));
        let per_dir: Vec<([A; N], [A; N], [A; N])> = (0..3).map(|d| self.staged3((lo[d].clone(), dd[d].clone(), upc[d].clone()))).collect();
        CellTerms3 {
            r,
            d0,
            lo: std::array::from_fn(|d| per_dir[d].0.clone()),
            dd: std::array::from_fn(|d| per_dir[d].1.clone()),
            up: std::array::from_fn(|d| per_dir[d].2.clone()),
        }
    }

    /// Kernel 5 (`turbulence3::solve_and_update`): the three-factor line solves (i, j, k; no k line on a wedge), the clipped update and the new eddy viscosity. Solid rows are the
    /// identity.
    pub(crate) fn solve_and_update3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, model: &M, dev: &TDev3<N>, grad: &[[A; 3]; N], ct: &CellTerms3<N>) -> TUpdate3<N> {
        let fluid = &tg.fluid;
        let mut x: [A; N] = std::array::from_fn(|e| select(fluid, &tg.vol * &ct.r[e], c(0.0)));
        for dir in 0..(if self.skip_k { 2 } else { 3 }) {
            let lo: [A; N] = std::array::from_fn(|e| if dir == 0 { ct.lo[0][e].clone() } else { &ct.lo[dir][e] / &ct.d0[e] });
            let up: [A; N] = std::array::from_fn(|e| if dir == 0 { ct.up[0][e].clone() } else { &ct.up[dir][e] / &ct.d0[e] });
            let d: [A; N] = std::array::from_fn(|e| select(fluid, if dir == 0 { &ct.d0[e] + &ct.dd[0][e] } else { c(1.0) + &ct.dd[dir][e] / &ct.d0[e] }, c(1.0)));
            let (lo, up) = (lo.map(|a| select(fluid, a, c(0.0))), up.map(|a| select(fluid, a, c(0.0))));
            x = tridiag3(dir, &lo, &d, &up, &x);
        }
        let turb: [A; N] = std::array::from_fn(|e| select(fluid, model.clip(e, &dev.turb[e], &(&dev.turb[e] + &x[e])), c(0.0)));
        let mu_t = select(fluid, model.eddy_viscosity(&self.cell_in3(dev, &turb, grad)), c(0.0));
        let n_active = c(tg.n_active as f64);
        let rnorm: [A; N] = std::array::from_fn(|e| (select(fluid, &ct.r[e] * &ct.r[e], c(0.0)).sum() / n_active).sqrt());
        TUpdate3 { turb, mu_t, rnorm }
    }

    /// One turbulence update from explicit device inputs (`turbulence3::update`): gradients, blends, face data, cell terms, solves and update.
    pub(crate) fn turbulence_update3<M: TurbModel3<N>, const N: usize>(&self, tg: &TGeo3, vg: &VGeo3, model: &M, dev: &TDev3<N>) -> TUpdate3<N> {
        let grad = self.scalar_gradients3(tg, vg, model, dev);
        let blend = self.blends3(tg, model, dev, &grad);
        let tf = self.face_data3(tg, vg, model, dev, &grad, &blend);
        let ct = self.cell_terms3(tg, model, dev, &grad, &blend, &tf);
        self.solve_and_update3(tg, model, dev, &grad, &ct)
    }

    // -----------------------------------------------------------------------------------------------------------------
    // host probe

    /// Uploads the arrays of the CPU `TurbIn` (the padded host primitives reduced to the cells; `dwall` clamped to a finite value, a non-finite primitive of a solid cell
    /// replaced by 1) as the device inputs of the update.
    fn upload_turb3<const N: usize>(&self, cx: &cpu3::TurbIn<N>) -> Result<(TDev3<N>, Rec3)> {
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let nc = ni * nj * nk;
        let shape = [ni, nj, nk];
        let ctx = &self.ctx;
        let fin = |x: f64| if x.is_finite() { x } else { 1.0 };
        let p: Rec3 = {
            let mut v: [Vec<f64>; 5] = std::array::from_fn(|_| Vec::with_capacity(nc));
            for c in 0..nc {
                let w = &cx.w[crate::solver3::pidx3(ni, nj, nk, (c % ni) as isize, ((c / ni) % nj) as isize, (c / (ni * nj)) as isize)];
                for q in 0..5 {
                    v[q].push(if cx.solid[c] { fin(w[q]) } else { w[q] });
                }
            }
            let mut a = Vec::with_capacity(5);
            for col in &v {
                a.push(up(ctx, col, shape)?);
            }
            a.try_into().unwrap_or_else(|_| unreachable!("five components"))
        };
        let per_cell = |f: &dyn Fn(usize) -> f64| up(ctx, &(0..nc).map(f).collect::<Vec<_>>(), shape);
        let grad: Grad12 = {
            let mut a = Vec::with_capacity(12);
            for q in 0..12 {
                a.push(per_cell(&|c| cx.grad[c][q])?);
            }
            a.try_into().unwrap_or_else(|_| unreachable!("twelve components"))
        };
        let mass = |dir: usize| -> Result<A> {
            let fl = [cx.fi, cx.fj, cx.fk][dir];
            up(ctx, &fl.iter().map(|f| f[0]).collect::<Vec<_>>(), cv3::face_shape(cx.grid, dir))
        };
        let dev = TDev3 {
            rho: p[0].clone(),
            cell: VCell3 { grad, mu: per_cell(&|c| cx.mu[c])? },
            mu_t: per_cell(&|c| cx.mu_t[c])?,
            wall: [per_cell(&|c| cx.wall[c][0])?, per_cell(&|c| cx.wall[c][1])?, per_cell(&|c| cx.wall[c][2])?],
            dwall: per_cell(&|c| cx.dwall[c].min(DWALL_MAX3))?,
            mass: [mass(0)?, mass(1)?, mass(2)?],
            dt: per_cell(&|c| cx.dt[c])?,
            turb: {
                let mut a = Vec::with_capacity(N);
                for e in 0..N {
                    a.push(per_cell(&|c| cx.turb[c][e])?);
                }
                a.try_into().unwrap_or_else(|_| unreachable!("N scalars"))
            },
        };
        Ok((dev, p))
    }

    /// The host geometry of the turbulence kernels of the grid of `cx` (`sym`: the symmetry sides of the domain box).
    fn geometry_turb3<const N: usize>(&self, cx: &cpu3::TurbIn<N>, sym: &cv3::SymSides) -> Result<(VGeo3, TGeo3)> {
        if (cx.grid.ni, cx.grid.nj, cx.grid.nk) != (self.ni, self.nj, self.nk) {
            return Err(luma::LumaError::Unsupported("the grid of the turbulence inputs differs from the grid of the solver".into()));
        }
        let visc = crate::solver::Viscous { re_c: 1.0, sutherland: 0.368, turbulence: crate::solver::Turbulence::Sst, wall_i_start: 0, roughness: 0.0 };
        let vg = VGeo3::new(&self.ctx, cx.grid, cx.solid, sym, visc)?;
        let tg = TGeo3::new(&self.ctx, cx.grid, cx.solid, sym, cx.bc, &vg)?;
        Ok((vg, tg))
    }

    /// Runs every stage of the update on the host arrays of the CPU `TurbIn` (`sym` the symmetry sides of the domain box, `ks` the roughness per cell, empty: all smooth)
    /// and returns every intermediate, downloaded in the layouts of the CPU functions: the wall state of the inputs `w`, `mu`, `dwall`, `ks`, then the stages 2 to 5 from the
    /// `cx.wall` given (the stages chain on the device: the gradients of the stage 1 feed the blends and so on). The kernel-level comparison with `crate::turbulence3`.
    pub fn turbulence_probe3<M: TurbModel3<N>, const N: usize>(&self, model: &M, cx: &cpu3::TurbIn<N>, sym: &cv3::SymSides, ks: &[f64]) -> Result<TurbProbe<N>> {
        let (vg, tg) = self.geometry_turb3(cx, sym)?;
        let (dev, p) = self.upload_turb3(cx)?;
        let nc = self.ni * self.nj * self.nk;
        let shape = [self.ni, self.nj, self.nk];
        let ks_a = if ks.is_empty() { None } else { Some(up(&self.ctx, ks, shape)?) };
        let wall_out = self.wall_state_pass3(&tg, &p, &dev.cell.mu, &dev.dwall, ks_a.as_ref())?;
        let grad = self.scalar_gradients3(&tg, &vg, model, &dev);
        let blend = self.blends3(&tg, model, &dev, &grad);
        let tf = self.face_data3(&tg, &vg, model, &dev, &grad, &blend);
        let ct = self.cell_terms3(&tg, model, &dev, &grad, &blend, &tf);
        let upd = self.solve_and_update3(&tg, model, &dev, &grad, &ct);
        let down = |a: &[A]| -> Result<Vec<Vec<f64>>> { a.iter().map(to_f64).collect() };
        let wall_cols = down(&wall_out)?;
        let grad_cols: Vec<Vec<f64>> = grad.iter().flat_map(|g| g.iter()).map(to_f64).collect::<Result<_>>()?;
        let (f1, x) = (to_f64(&blend.f1)?, to_f64(&blend.x)?);
        let faces: [Vec<cpu3::TFace<N>>; 3] = {
            let one = |t: &TFace3<N>| -> Result<Vec<cpu3::TFace<N>>> {
                let m = to_f64(&t.m)?;
                let (g, fl, l, r) = (down(&t.g)?, down(&t.flux)?, down(&t.l)?, down(&t.r)?);
                Ok((0..m.len())
                    .map(|q| cpu3::TFace { m: m[q], g: std::array::from_fn(|e| g[e][q]), flux: std::array::from_fn(|e| fl[e][q]), l: std::array::from_fn(|e| l[e][q]), r: std::array::from_fn(|e| r[e][q]) })
                    .collect())
            };
            [one(&tf[0])?, one(&tf[1])?, one(&tf[2])?]
        };
        let (r, d0) = (down(&ct.r)?, down(&ct.d0)?);
        let fam = |a: &[[A; N]; 3]| -> Result<Vec<Vec<Vec<f64>>>> { a.iter().map(|d| down(d)).collect() };
        let (lo, dd, upc) = (fam(&ct.lo)?, fam(&ct.dd)?, fam(&ct.up)?);
        let terms = (0..nc)
            .map(|c| cpu3::CellTerms {
                r: std::array::from_fn(|e| r[e][c]),
                d0: std::array::from_fn(|e| d0[e][c]),
                lo: std::array::from_fn(|d| std::array::from_fn(|e| lo[d][e][c])),
                dd: std::array::from_fn(|d| std::array::from_fn(|e| dd[d][e][c])),
                up: std::array::from_fn(|d| std::array::from_fn(|e| upc[d][e][c])),
            })
            .collect();
        let (turb, mu_t, rnorm) = (down(&upd.turb)?, to_f64(&upd.mu_t)?, down(&upd.rnorm)?);
        Ok(TurbProbe {
            wall: (0..nc).map(|c| [wall_cols[0][c], wall_cols[1][c], wall_cols[2][c]]).collect(),
            tg: (0..nc).map(|c| std::array::from_fn(|e| std::array::from_fn(|a| grad_cols[3 * e + a][c]))).collect(),
            blend: (0..nc).map(|c| cpu3::Blend { f1: f1[c], x: x[c] }).collect(),
            faces,
            terms,
            update: cpu3::Update { turb: (0..nc).map(|c| std::array::from_fn(|e| turb[e][c])).collect(), mu_t, rnorm: std::array::from_fn(|e| rnorm[e][0]) },
        })
    }

    /// Kernel launches and GPU time of the wall state pass and of one full turbulence update (stages 1 to 5, the three-factor line solves included) on the inputs of
    /// the CPU `TurbIn`: the device inputs are uploaded first, then each part is run once to compile, both run for half a second to bring the GPU to its working clock, and then `reps` times under the profiler; the run of the least GPU time is
    /// returned (the GPU is shared: a run can be slowed by other work). `(wall pass, update)`.
    pub fn turbulence_profile3<M: TurbModel3<N>, const N: usize>(&self, model: &M, cx: &cpu3::TurbIn<N>, sym: &cv3::SymSides, ks: &[f64], reps: usize) -> Result<(luma::Profile, luma::Profile)> {
        let (vg, tg) = self.geometry_turb3(cx, sym)?;
        let (dev, p) = self.upload_turb3(cx)?;
        let ks_a = if ks.is_empty() { None } else { Some(up(&self.ctx, ks, [self.ni, self.nj, self.nk])?) };
        let wall_run = || -> Result<()> {
            self.wall_state_pass3(&tg, &p, &dev.cell.mu, &dev.dwall, ks_a.as_ref())?.eval()?;
            self.ctx.sync()
        };
        let update_run = || -> Result<()> {
            let u = self.turbulence_update3(&tg, &vg, model, &dev);
            (u.turb.clone(), u.mu_t.clone(), u.rnorm.clone()).eval()?;
            self.ctx.sync()
        };
        wall_run()?;
        update_run()?;
        // the GPU idles at a low clock: half a second of the update before anything is measured
        let t0 = std::time::Instant::now();
        while t0.elapsed().as_secs_f64() < 0.5 {
            update_run()?;
            wall_run()?;
        }
        let ctx = self.ctx.clone();
        let best = |run: &dyn Fn() -> Result<()>| -> Result<luma::Profile> {
            let mut best: Option<luma::Profile> = None;
            for _ in 0..reps.max(1) {
                let (out, profile) = ctx.profile(run)?;
                out?;
                if best.as_ref().is_none_or(|b| profile.total() < b.total()) {
                    best = Some(profile);
                }
            }
            Ok(best.expect("at least one run"))
        };
        Ok((best(&wall_run)?, best(&update_run)?))
    }
}

/// The wall distance is clamped to this value on upload (`walldist3` returns `inf` in solid cells and without a wall; `d^2` must stay finite in f32).
const DWALL_MAX3: f64 = 1e10;

/// Every stage of a turbulence update, downloaded to f64 in the layouts of the CPU `crate::turbulence3` functions (`LazySolver3::turbulence_probe3`).
pub struct TurbProbe<const N: usize> {
    /// `wall_state_pass`: `[k_w, omega_w, mu_t,w]` per cell (zero without a no-slip face).
    pub wall: Vec<[f64; 3]>,
    /// `scalar_gradients`.
    pub tg: Vec<cpu3::SGrad<N>>,
    /// `blends`.
    pub blend: Vec<cpu3::Blend>,
    /// `face_data`.
    pub faces: [Vec<cpu3::TFace<N>>; 3],
    /// `cell_terms`.
    pub terms: Vec<cpu3::CellTerms<N>>,
    /// `solve_and_update`.
    pub update: cpu3::Update<N>,
}
