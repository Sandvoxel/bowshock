// The viscous terms of the axisymmetric compressible Navier-Stokes equations in array ops. This file is `include!`d by `lib.rs` after
// `body.rs` and `implicit.rs` into the same per-precision module (see `body.rs` for what that module defines).
//
// It mirrors `bowshock_core::viscous` (`cell_pass`, `face_pass`, `hoop_stress`, `diffusion_rate`, `face_lambdas`, `Walls`) and the viscous
// parts of `bowshock_core::solver` (`Viscous`, `wall_ghost`, the residual `F - Fv` with the hoop stress, the diffusive time step). Units:
// `rho_inf = c_inf = 1`, `p_inf = 1 / gamma`, `T = gamma p / rho`, lengths in nose diameters,
//
//     mu(T)  = (1 / re_c) T^1.5 (1 + S) / (T + S)                       S = sutherland
//     kappa  = (mu / PRANDTL + mu_t / PRANDTL_T) / (gamma - 1)          q = -kappa grad T
//
// The residual of a cell uses the face flux `F - Fv` and the radial-momentum source `(p - t_tt) A / V`; see `LazySolver::viscous_flow`.
// Everything here is in the precision `S` except the left-hand side of the implicit scheme (`two_lambda` and the diffusion rates of the time step, converted by `lhs`).
//
// A state across a face is a slice of the padded primitive record `w` (the domain ghost, the block ghost below a surface); across the
// base wall of the block it is the wall ghost of the first fluid cell. A ghost carries the gradient, viscosity and eddy viscosity of its
// fluid cell (`cell_sides`) and sits at the mirror image of that cell's centre in the face centre (the geometry arrays `VDir`).

use bowshock_core::solver::{Turbulence, Viscous, PRANDTL, PRANDTL_T};
use bowshock_core::turbulence::{omega_inf, wall_distances, K_INF};

/// One residual evaluation: the residual, the time step, the primitives and padded primitives of the evaluated state, the mass flow
/// density `F[0]` of the faces (what the turbulence transport convects with) and the viscous quantities.
struct Stage {
    res: Rec,
    dt: Option<A>,
    p: Rec,
    w: Rec,
    mass: [A; 2],
    vf: Option<VFlow>,
}

/// Viscous quantities of one residual evaluation (evaluated arrays): `[u_x, u_r, v_x, v_r, T_x, T_r, mu, mu_t]` of every cell and
/// the viscous face fluxes `Fv.n` (momentum and energy components; the mass component is zero) of the i faces and j faces.
struct VFlow {
    cell: [A; 8],
    fvi: [A; 3],
    fvj: [A; 3],
}

/// Geometry of the compact face gradients, per direction (i faces `[ni + 1, nj]`, j faces `[ni, nj + 1]`): the unit vector `e` and the
/// inverse of the distance `|d|` between the two sides of the face (a ghost at the mirror image of its cell centre in the face
/// centre), the face radius (1 on the axis, where the face weight is zero anyway) and the no-slip mask of the faces.
struct VDir {
    ex: A,
    er: A,
    inv_d: A,
    rm: A,
    noslip: Array<u32>,
    /// `1 / |d|` in f32 (left-hand side).
    inv_d_f: F,
}

/// Everything a viscous run needs beyond the Euler solver, uploaded once from the grid by `LazySolver::set_viscous`.
struct VGeo {
    v: Viscous,
    dir: [VDir; 2],
    /// `1 / A`, `V`, the cell radius and zeros, `[ni, nj]`.
    inv_a: A,
    vol: A,
    rc: A,
    zero: A,
    /// Domain `jmin` wall (no block): the no-slip columns as a mask of the two ghost layers `[ni, 2]` and of an edge row `[ni, 1]`.
    jmin_ghost: Option<Array<u32>>,
    jmin_cols: Option<Array<u32>>,
    /// With a block: the no-slip columns of the two block-ghost groups (body surface, sting surface), masks `[i1 - i0, 2]`.
    block_ghost: Option<[Array<u32>; 2]>,
    /// With a block: the first fluid cell of a line behind the solid sees a slip wall (`[0]`) or a no-slip wall (`[1]`); per direction.
    fold: Option<[[Array<u32>; 2]; 2]>,
    /// The wall distance of every cell centroid (`f64::INFINITY` capped to `1e10`) of the SST model.
    dwall: Option<A>,
}

/// `Walls::noslip_jface`: is the j-face `(i, j)` a no-slip wall?
fn noslip_jface(v: &Viscous, jmin_slip: bool, blk: Option<Block>, i: usize, j: usize) -> bool {
    match blk {
        None => j == 0 && jmin_slip && i >= v.wall_i_start,
        Some(b) => (j == b.j_end && i >= v.wall_i_start && i < b.i_end) || (j == b.j_sting && b.j_sting > 0 && i >= b.i_end),
    }
}

/// `Walls::noslip_iface`: is the i-face `(i, j)` the no-slip base wall?
fn noslip_iface(blk: Option<Block>, i: usize, j: usize) -> bool {
    blk.is_some_and(|b| i == b.i_end && j >= b.j_sting && j < b.j_end)
}

fn mask_array(ctx: &Context, shape: [usize; 2], f: &dyn Fn(usize, usize) -> bool) -> Result<Array<u32>> {
    let v: Vec<u32> = (0..shape[1]).flat_map(|j| (0..shape[0]).map(move |i| (i, j))).map(|(i, j)| u32::from(f(i, j))).collect();
    Array::from_slice_on(ctx, &v, shape)
}

/// `(e_x, e_r, 1 / |d|, r_m)` of every face of direction `dir` as `bowshock_core::viscous::stencil` computes them (`iface_sides`,
/// `jface_sides`, `side_pos`): the two sides of a face are the adjacent cells, a ghost side is the mirror image of its cell centre in the
/// face centre. Faces inside the solid block get `(1, 0, 1, 1)`. `r_m` is 1 where the face radius is 0 (the viscous face flux then
/// carries the weight 0 anyway; it avoids `v / 0`).
fn face_stencils(grid: &Grid, blk: Option<Block>, dir: usize) -> Vec<[f64; 4]> {
    let (ni, nj) = (grid.ni, grid.nj);
    let shape = if dir == 0 { [ni + 1, nj] } else { [ni, nj + 1] };
    let mut out = Vec::with_capacity(shape[0] * shape[1]);
    for j in 0..shape[1] {
        for i in 0..shape[0] {
            // (cell, ghost) of the lower and upper side; `None` inside the solid block
            let sides: Option<[(usize, bool); 2]> = if dir == 0 {
                'sides: {
                    if let Some(b) = blk {
                        if j < b.j_end {
                            if j < b.j_sting || i < b.i_end {
                                break 'sides None;
                            }
                            if i == b.i_end {
                                let c = j * ni + i;
                                break 'sides Some([(c, true), (c, false)]);
                            }
                        }
                    }
                    Some(if i == 0 {
                        let c = j * ni;
                        [(c, true), (c, false)]
                    } else if i == ni {
                        let c = j * ni + ni - 1;
                        [(c, false), (c, true)]
                    } else {
                        [(j * ni + i - 1, false), (j * ni + i, false)]
                    })
                }
            } else {
                'sides: {
                    if let Some(b) = blk {
                        let j0 = b.col_start(i);
                        if j < j0 {
                            break 'sides None;
                        }
                        if j == j0 && j0 > 0 {
                            let c = j * ni + i;
                            break 'sides Some([(c, true), (c, false)]);
                        }
                    }
                    Some(if j == 0 {
                        [(i, true), (i, false)]
                    } else if j == nj {
                        let c = (nj - 1) * ni + i;
                        [(c, false), (c, true)]
                    } else {
                        [((j - 1) * ni + i, false), (j * ni + i, false)]
                    })
                }
            };
            let (face, (a, b)) = if dir == 0 {
                (&grid.iface[j * (ni + 1) + i], (grid.node(i, j), grid.node(i, j + 1)))
            } else {
                (&grid.jface[j * ni + i], (grid.node(i, j), grid.node(i + 1, j)))
            };
            let (xf, rf) = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
            out.push(match sides {
                None => [1.0, 0.0, 1.0, 1.0],
                Some([l, r]) => {
                    let pos = |(c, ghost): (usize, bool)| {
                        let (x, rr) = (grid.xc[c], grid.rc[c]);
                        if ghost {
                            (2.0 * xf - x, 2.0 * rf - rr)
                        } else {
                            (x, rr)
                        }
                    };
                    let ((xl, rl), (xr, rr)) = (pos(l), pos(r));
                    let (dx, dr) = (xr - xl, rr - rl);
                    let dd = dx.hypot(dr);
                    [dx / dd, dr / dd, 1.0 / dd, if face.rm == 0.0 { 1.0 } else { face.rm }]
                }
            });
        }
    }
    out
}

impl VGeo {
    fn new(ctx: &Context, grid: &Grid, cfg: &Config, blk: Option<Block>, v: Viscous) -> Result<VGeo> {
        let (ni, nj) = (grid.ni, grid.nj);
        let jmin_slip = cfg.bc[2] == Bc::SlipWall;
        let dir = |d: usize| -> Result<VDir> {
            let shape = if d == 0 { [ni + 1, nj] } else { [ni, nj + 1] };
            let st = face_stencils(grid, blk, d);
            let col = |k: usize| st.iter().map(|s| s[k]).collect::<Vec<_>>();
            Ok(VDir {
                ex: up(ctx, &col(0), shape)?,
                er: up(ctx, &col(1), shape)?,
                inv_d: up(ctx, &col(2), shape)?,
                rm: up(ctx, &col(3), shape)?,
                noslip: mask_array(ctx, shape, &|i, j| if d == 0 { noslip_iface(blk, i, j) } else { noslip_jface(&v, jmin_slip, blk, i, j) })?,
                inv_d_f: up_lhs(ctx, &col(2), shape)?,
            })
        };
        let first_refl = |d: usize, i: usize, j: usize| match (blk, d) {
            (Some(b), 0) => b.row_start(j) == Some(i) && i > 0,
            (Some(b), _) => b.col_start(i) == j && j > 0,
            (None, _) => false,
        };
        let noslip_fold =
            |d: usize, i: usize, j: usize| first_refl(d, i, j) && if d == 0 { noslip_iface(blk, i, j) } else { noslip_jface(&v, jmin_slip, blk, i, j) };
        let dwall = (v.turbulence == Turbulence::Sst)
            .then(|| -> Result<A> { up(ctx, &wall_distances(grid, v, &cfg.bc, blk).iter().map(|d| d.min(1e10)).collect::<Vec<_>>(), [ni, nj]) })
            .transpose()?;
        Ok(VGeo {
            v,
            dir: [dir(0)?, dir(1)?],
            inv_a: up(ctx, &grid.area.iter().map(|a| 1.0 / a).collect::<Vec<_>>(), [ni, nj])?,
            vol: up(ctx, &grid.vol, [ni, nj])?,
            rc: up(ctx, &grid.rc, [ni, nj])?,
            zero: up(ctx, &vec![0.0; ni * nj], [ni, nj])?,
            jmin_ghost: if blk.is_none() { Some(mask_array(ctx, [ni, 2], &|i, _| noslip_jface(&v, jmin_slip, None, i, 0))?) } else { None },
            jmin_cols: if blk.is_none() { Some(mask_array(ctx, [ni, 1], &|i, _| noslip_jface(&v, jmin_slip, None, i, 0))?) } else { None },
            block_ghost: match blk {
                Some(b) => Some([
                    mask_array(ctx, [b.i_end, 2], &|i, _| noslip_jface(&v, jmin_slip, blk, i, b.j_end))?,
                    mask_array(ctx, [ni - b.i_end, 2], &|i, _| noslip_jface(&v, jmin_slip, blk, b.i_end + i, b.j_sting))?,
                ]),
                None => None,
            },
            fold: match blk {
                Some(_) => Some([
                    [
                        mask_array(ctx, [ni, nj], &|i, j| first_refl(0, i, j) && !noslip_fold(0, i, j))?,
                        mask_array(ctx, [ni, nj], &|i, j| noslip_fold(0, i, j))?,
                    ],
                    [
                        mask_array(ctx, [ni, nj], &|i, j| first_refl(1, i, j) && !noslip_fold(1, i, j))?,
                        mask_array(ctx, [ni, nj], &|i, j| noslip_fold(1, i, j))?,
                    ],
                ]),
                None => None,
            },
            dwall,
        })
    }
}

/// The wall ghost `(rho, -u, -v, p)` of a no-slip wall (`bowshock_core::viscous::wall_ghost` with `noslip = true`).
fn noslip_ghost(w: &Rec) -> Rec {
    [w[0].clone(), -&w[1], -&w[2], w[3].clone()]
}

/// `wall_ghost`: the no-slip ghost where `noslip` is 1, else the reflection about the face normal `(n_x, n_r)`; `None`: the reflection
/// everywhere.
fn wall_ghost(w: &Rec, nx: &A, nr: &A, noslip: Option<&Array<u32>>) -> Rec {
    match noslip {
        None => reflect(w, nx, nr),
        Some(m) => {
            let (n, r) = (noslip_ghost(w), reflect(w, nx, nr));
            [r[0].clone(), select(m, &n[1], &r[1]), select(m, &n[2], &r[2]), r[3].clone()]
        }
    }
}

/// The two forms of the `Staging` cache applied to a record: evaluated, or left lazy for `Staging::Fusion`.
fn staged<F: Fields>(staging: Staging, x: F) -> F {
    if staging == Staging::Fusion {
        x
    } else {
        x.cache()
    }
}

/// `T = gamma p / rho`.
fn temperature(w: &Rec) -> A {
    c(GAMMA) * &w[3] / &w[0]
}

impl LazySolver {
    /// Switches the viscous terms on (`Some`, with the SST model if `v.turbulence` says so) or off (`None`, the Euler equations). `grid` is the
    /// grid of the solver: the geometry of the compact face gradients, the cell metrics and the wall distance are computed from it in f64 and
    /// uploaded in the precision `S`. Declare the solid block first; the turbulence state restarts at the free stream (the mean flow is not
    /// touched).
    pub fn set_viscous(&mut self, v: Option<Viscous>, grid: &Grid) -> Result<()> {
        assert_eq!((grid.ni, grid.nj), (self.ni, self.nj), "set_viscous: the grid of the solver");
        self.visc = v.map(|v| VGeo::new(&self.ctx, grid, &self.cfg, self.block, v)).transpose()?;
        let u = self.state_f64()?;
        self.init_turb(&u)
    }

    /// Viscous-flow parameters, `None` for the Euler equations.
    pub fn viscous(&self) -> Option<Viscous> {
        self.visc.as_ref().map(|g| g.v)
    }

    /// Is the SST turbulence model active?
    fn sst(&self) -> bool {
        self.visc.as_ref().is_some_and(|g| g.v.turbulence == Turbulence::Sst)
    }

    /// The eddy viscosity of the loop state (`Some` for the SST model).
    fn state_mu_t(&self) -> Option<A> {
        self.sst().then(|| self.state.as_ref().expect("state")[8].clone())
    }

    /// The turbulence slots of the loop state at the free stream of the conserved state `u`: `k_inf`, `omega_inf`, the eddy viscosity
    /// `rho k / omega` (0 in solid cells) and a zero `k` residual; one-element dummies without the SST model.
    fn init_turb(&mut self, u: &[[f64; 4]]) -> Result<()> {
        let n = self.ni * self.nj;
        let shape = [self.ni, self.nj];
        let slots = match &self.visc {
            Some(g) if g.v.turbulence == Turbulence::Sst => {
                let om = omega_inf(g.v.re_c);
                let mu_t: Vec<f64> =
                    (0..n).map(|c| if self.block.is_some_and(|b| b.solid(c % self.ni, c / self.ni)) { 0.0 } else { u[c][0] * K_INF / om }).collect();
                [up(&self.ctx, &vec![K_INF; n], shape)?, up(&self.ctx, &vec![om; n], shape)?, up(&self.ctx, &mu_t, shape)?, up(&self.ctx, &[0.0], 1)?]
            }
            _ => [up(&self.ctx, &[0.0], 1)?, up(&self.ctx, &[0.0], 1)?, up(&self.ctx, &[0.0], 1)?, up(&self.ctx, &[0.0], 1)?],
        };
        if let Some(s) = &mut self.state {
            let [k, om, mt, tr] = slots;
            (s[6], s[7], s[8], s[9]) = (k, om, mt, tr);
        }
        Ok(())
    }

    /// Overwrites the turbulence state of an SST run: `[k, omega]` and the eddy viscosity of every cell (`Solver::turb`, `Solver::mu_t`).
    pub fn set_turb(&mut self, turb: &[[f64; 2]], mu_t: &[f64]) -> Result<()> {
        assert!(self.sst(), "set_turb needs the SST model");
        let shape = [self.ni, self.nj];
        let (k, om) =
            (up(&self.ctx, &turb.iter().map(|t| t[0]).collect::<Vec<_>>(), shape)?, up(&self.ctx, &turb.iter().map(|t| t[1]).collect::<Vec<_>>(), shape)?);
        let mt = up(&self.ctx, mu_t, shape)?;
        let s = self.state.as_mut().expect("state");
        (s[6], s[7], s[8]) = (k, om, mt);
        Ok(())
    }

    /// `([k, omega], mu_t)` of every cell on the host (SST runs; `Solver::turb`, `Solver::mu_t`).
    pub fn turb_state_f64(&self) -> Result<(Vec<[f64; 2]>, Vec<f64>)> {
        assert!(self.sst(), "turb_state_f64 needs the SST model");
        let s = self.state.as_ref().expect("state");
        let (k, om, mt) = (to_f64(&s[6])?, to_f64(&s[7])?, to_f64(&s[8])?);
        Ok((k.iter().zip(&om).map(|(&k, &o)| [k, o]).collect(), mt))
    }

    /// The density residual norm of `k` of the last turbulence update (`Solver::turb_residual`; 0 before the first).
    pub fn turb_residual(&self) -> Result<f64> {
        Ok(to_f64(&self.state.as_ref().expect("state")[9])?[0])
    }

    /// The ghost state below / behind a slip-wall boundary: the reflection, or on the no-slip columns of the viscous `jmin` wall
    /// (`Walls::noslip_jface(i, 0)`) the wall ghost `(rho, -u, -v, p)`.
    fn slip_wall_ghost(&self, side: usize, mirror: &Rec, nx: &A, nr: &A) -> Rec {
        match self.visc.as_ref().and_then(|g| g.jmin_ghost.as_ref()) {
            Some(m) if side == 2 => wall_ghost(mirror, nx, nr, Some(m)),
            _ => reflect(mirror, nx, nr),
        }
    }

    /// The ghost of the solid cells below the body surface (`i0..i1 = 0..i_end`) or the sting surface (`i_end..ni`): the wall ghost of a no-slip
    /// column, else the reflection.
    fn block_wall_ghost(&self, src: &Rec, nx: &A, nr: &A, i0: usize) -> Rec {
        match (&self.visc, self.block) {
            (Some(g), Some(b)) => wall_ghost(src, nx, nr, g.block_ghost.as_ref().map(|m| &m[usize::from(i0 == b.i_end)])),
            _ => reflect(src, nx, nr),
        }
    }

    /// The ghost across the base wall of the block (`reflect` for the Euler equations, `(rho, -u, -v, p)` for a viscous run: every base-wall
    /// face is no-slip).
    fn base_wall_ghost(&self, w: &Rec, nx: &A, nr: &A) -> Rec {
        if self.visc.is_some() {
            noslip_ghost(w)
        } else {
            reflect(w, nx, nr)
        }
    }

    /// The two sides `(l, r)` (states below and above) of every face of direction `dir` as primitive records of the face shape: slices of
    /// the padded record `w` (domain ghosts, block ghosts), and across the base wall of the block the wall ghost of the first fluid cell
    /// (`iface_sides`, `jface_sides`).
    fn face_sides(&self, w: &Rec, dir: usize) -> (Rec, Rec) {
        let (ni, nj) = (self.ni, self.nj);
        if dir == 0 {
            let wi = w.slice(1, 2..nj + 2);
            let (l, r) = (wi.slice(0, 1..ni + 2), wi.slice(0, 2..ni + 3));
            // across the base wall: the wall ghost of the first fluid cell (the padded state there is solid)
            let l = match &self.lb {
                Some(lb) => Rec::select(&lb.base, &noslip_ghost(&r), &l),
                None => l,
            };
            (l, r)
        } else {
            let wj = w.slice(0, 2..ni + 2);
            (wj.slice(1, 1..nj + 2), wj.slice(1, 2..nj + 3))
        }
    }

    /// Cell data `a` (`[ni, nj]` fields) on the two sides of every face of direction `dir`: the cell below and above, and a ghost side carries
    /// the data of its fluid cell (the edge cell at the domain boundary; across a wall of the block the cell on the fluid side).
    fn cell_sides<const N: usize>(&self, a: &[A; N], dir: usize) -> ([A; N], [A; N]) {
        let ni = self.ni;
        let (mut l, r): ([A; N], [A; N]) = if dir == 0 {
            (a.pad([(1, 0)], PadMode::Edge), a.pad([(0, 1)], PadMode::Edge))
        } else {
            (a.pad([(0, 0), (1, 0)], PadMode::Edge), a.pad([(0, 0), (0, 1)], PadMode::Edge))
        };
        if let Some(b) = self.block {
            let mut walls = vec![];
            if dir == 0 {
                walls.push((b.i_end..b.i_end + 1, b.j_sting..b.j_end));
            } else {
                walls.push((0..b.i_end, b.j_end..b.j_end + 1));
                if b.j_sting > 0 {
                    walls.push((b.i_end..ni, b.j_sting..b.j_sting + 1));
                }
            }
            for (ri, rj) in walls {
                let src = r.slice(0, ri.clone()).slice(1, rj.clone());
                l.slice_mut(0, ri).slice(1, rj).assign(src).expect("wall side assign");
            }
        }
        (l, r)
    }

    /// The viscous quantities of the state with primitives `p` and padded primitives `w` (`None` for the Euler equations); `mu_t` is the eddy
    /// viscosity of the SST model (`None`: zero).
    ///
    /// ```text
    /// mu(T) = T^1.5 (1 + S) / (T + S) / re_c,     g_c = (1/A) sum_faces s phi_f n len,   phi_f = (phi_c + phi_neighbour) / 2   (phi = u, v, T)
    /// g_f = gbar + ((phi_R - phi_L) / |d| - gbar.e) e,     gbar = (g_L + g_R) / 2,     e = d / |d|
    /// div = u_x + v_r + v_f / r_f,   mue = mu_f + mu_t,f,   txx = mue (2 u_x - 2/3 div),   trr = mue (2 v_r - 2/3 div),   txr = mue (u_r + v_x)
    /// Fv.n = [0, txx n_x + txr n_r, txr n_x + trr n_r, (u txx + v txr + kappa T_x) n_x + (u txr + v trr + kappa T_r) n_r]
    /// ```
    ///
    /// The eddy viscosity of a face on a no-slip wall is 0 (the ghost carries the negated eddy viscosity of its cell).
    fn viscous_flow(&self, p: &Rec, w: &Rec, mu_t: Option<&A>) -> Option<VFlow> {
        let vg = self.visc.as_ref()?;
        let v = vg.v;
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.geo;
        // pass 1: viscosity and the Green-Gauss gradient of every cell
        let t = temperature(p);
        let mu = (&t * t.sqrt()) * c(1.0 + v.sutherland) / (&t + c(v.sutherland)) / c(v.re_c);
        let (li, ri) = self.face_sides(w, 0);
        let (lj, rj) = self.face_sides(w, 1);
        let phi = |w: &Rec| [w[1].clone(), w[2].clone(), temperature(w)];
        let pc = phi(p);
        // (neighbour state, face normal, face length, sign) of the right, left, top and bottom face
        let nbs = [
            (phi(&ri.slice(0, 1..ni + 1)), g.inx.slice(0, 1..ni + 1), g.inr.slice(0, 1..ni + 1), g.ilen.slice(0, 1..ni + 1), true),
            (phi(&li.slice(0, 0..ni)), g.inx.slice(0, 0..ni), g.inr.slice(0, 0..ni), g.ilen.slice(0, 0..ni), false),
            (phi(&rj.slice(1, 1..nj + 1)), g.jnx.slice(1, 1..nj + 1), g.jnr.slice(1, 1..nj + 1), g.jlen.slice(1, 1..nj + 1), true),
            (phi(&lj.slice(1, 0..nj)), g.jnx.slice(1, 0..nj), g.jnr.slice(1, 0..nj), g.jlen.slice(1, 0..nj), false),
        ];
        let grad: [A; 6] = std::array::from_fn(|k| {
            let (m, comp) = (k / 2, k % 2);
            let mut acc: Option<A> = None;
            for (pn, nx, nr, len, plus) in &nbs {
                let pf = c(0.5) * (&pc[m] + &pn[m]);
                let n = if comp == 0 { nx } else { nr };
                let term = &pf * n * len;
                acc = Some(match (acc, plus) {
                    (None, true) => term,
                    (None, false) => -term,
                    (Some(a), true) => a + term,
                    (Some(a), false) => a - term,
                });
            }
            acc.expect("four faces") * &vg.inv_a
        });
        let [g0, g1, g2, g3, g4, g5] = grad;
        let mu_t = mu_t.cloned().unwrap_or_else(|| vg.zero.clone());
        let cell = staged(self.staging, [g0, g1, g2, g3, g4, g5, mu, mu_t]);
        // pass 2: viscous fluxes of the faces
        let fvi = staged(self.staging, self.viscous_face_flux(vg, 0, &li, &ri, &cell));
        let fvj = staged(self.staging, self.viscous_face_flux(vg, 1, &lj, &rj, &cell));
        Some(VFlow { cell, fvi, fvj })
    }

    /// `Fv.n` (components 1 to 3) of every face of direction `dir` from the states `wl`, `wr` on the two sides and the cell record
    /// `cell` (see [`LazySolver::viscous_flow`]).
    fn viscous_face_flux(&self, vg: &VGeo, dir: usize, wl: &Rec, wr: &Rec, cell: &[A; 8]) -> [A; 3] {
        let g = &self.geo;
        let d = &vg.dir[dir];
        let (nx, nr) = if dir == 0 { (&g.inx, &g.inr) } else { (&g.jnx, &g.jnr) };
        let (cl, cr) = self.cell_sides(cell, dir);
        let phi = |w: &Rec| [w[1].clone(), w[2].clone(), temperature(w)];
        let (pl, pr) = (phi(wl), phi(wr));
        // the corrected face gradient of (u, v, T)
        let mut gf: Vec<A> = Vec::with_capacity(6);
        for m in 0..3 {
            let (gbx, gbr) = (c(0.5) * (&cl[2 * m] + &cr[2 * m]), c(0.5) * (&cl[2 * m + 1] + &cr[2 * m + 1]));
            let corr = (&pr[m] - &pl[m]) * &d.inv_d - (&gbx * &d.ex + &gbr * &d.er);
            gf.push(&gbx + &corr * &d.ex);
            gf.push(&gbr + &corr * &d.er);
        }
        let mu_f = c(0.5) * (&cl[6] + &cr[6]);
        let mut_f = c(0.5) * (&cl[7] + &cr[7]);
        let mut_f = if vg.v.turbulence == Turbulence::Sst { select(&d.noslip, c(0.0), &mut_f) } else { mut_f };
        let (uf, vf) = (c(0.5) * (&wl[1] + &wr[1]), c(0.5) * (&wl[2] + &wr[2]));
        let (u_x, u_r, v_x, v_r, t_x, t_r) = (&gf[0], &gf[1], &gf[2], &gf[3], &gf[4], &gf[5]);
        let div = u_x + v_r + &vf / &d.rm;
        let mue = &mu_f + &mut_f;
        let txx = &mue * (c(2.0) * u_x - c(2.0 / 3.0) * &div);
        let trr = &mue * (c(2.0) * v_r - c(2.0 / 3.0) * &div);
        let txr = &mue * (u_r + v_x);
        let kappa = (&mu_f / c(PRANDTL) + &mut_f / c(PRANDTL_T)) / c(GAMMA - 1.0);
        [&txx * nx + &txr * nr, &txr * nx + &trr * nr, (&uf * &txx + &vf * &txr + &kappa * t_x) * nx + (&uf * &txr + &vf * &trr + &kappa * t_r) * nr]
    }

    /// The face flux `F - Fv` of a viscous run (the mass component is unchanged); the Euler flux without viscous terms.
    fn total_flux(&self, f: &Rec, fv: Option<&[A; 3]>) -> Rec {
        match fv {
            Some(fv) => [f[0].clone(), &f[1] - &fv[0], &f[2] - &fv[1], &f[3] - &fv[2]],
            None => f.clone(),
        }
    }

    /// The radial-momentum source `(p - t_tt) A / V` of a viscous run, `t_tt = (mu_c + mu_t,c) (2 v_c / r_c - 2/3 div_c)`,
    /// `div_c = u_x + v_r + v_c / r_c` (`hoop_stress`); `p A / V` for the Euler equations.
    fn hoop_source(&self, p: &Rec, vf: Option<&VFlow>) -> A {
        let g = &self.geo;
        match (&self.visc, vf) {
            (Some(vg), Some(vf)) => {
                let div = &vf.cell[0] + &vf.cell[3] + &p[2] / &vg.rc;
                let t_tt = (&vf.cell[6] + &vf.cell[7]) * (c(2.0) * &p[2] / &vg.rc - c(2.0 / 3.0) * &div);
                (&p[3] - &t_tt) * &g.area * &g.inv_v
            }
            _ => &p[3] * &g.area * &g.inv_v,
        }
    }

    /// Diffusive contribution `2 lv len^2 / A` to the spectral radius of the i and j direction (`diffusion_rate`), `len` the mean length of the
    /// two faces of the direction, `lv = gamma (mu / PRANDTL + mu_t / PRANDTL_T) / rho`.
    fn diffusion_rates(&self, vf: &VFlow, rho: &A) -> (A, A) {
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.geo;
        let lv = c(GAMMA) * (&vf.cell[6] / c(PRANDTL) + &vf.cell[7] / c(PRANDTL_T)) / rho;
        let li = c(0.5) * (g.ilen.slice(0, 0..ni) + g.ilen.slice(0, 1..ni + 1));
        let lj = c(0.5) * (g.jlen.slice(1, 0..nj) + g.jlen.slice(1, 1..nj + 1));
        (c(2.0) * &lv * &li * &li / &g.area, c(2.0) * &lv * &lj * &lj / &g.area)
    }

    /// `2 lam` of every face of direction `dir` (f32), `lam = gamma (mu_f / PRANDTL + mu_t,f / PRANDTL_T) / (rho_f |d|)` (`face_lambdas`),
    /// `rho_f` the mean of the densities `rho_l`, `rho_r` of the two sides: the diffusive spectral radius that the implicit operator adds to
    /// the dissipation matrix of the face (`2 lam I`).
    fn two_lambda(&self, vf: &VFlow, dir: usize, rho_l: &F, rho_r: &F) -> F {
        let vg = self.visc.as_ref().expect("viscous run");
        let (cl, cr) = self.cell_sides(&[vf.cell[6].clone(), vf.cell[7].clone()], dir);
        let mu_f = lhs(&(c(0.5) * (&cl[0] + &cr[0])));
        let mut_f = lhs(&(c(0.5) * (&cl[1] + &cr[1])));
        let mut_f = if vg.v.turbulence == Turbulence::Sst { select(&vg.dir[dir].noslip, 0.0f32, &mut_f) } else { mut_f };
        let rho_f = 0.5f32 * (rho_l + rho_r);
        cf(2.0 * GAMMA) * (mu_f / cf(PRANDTL) + mut_f / cf(PRANDTL_T)) * &vg.dir[dir].inv_d_f / rho_f
    }

    /// The wall-ghost Jacobian `dU_ghost / dU_interior` of a slip-wall boundary `side`: the reflection, and on the no-slip columns of the viscous
    /// `jmin` wall `diag(1, -1, -1, 1)` (`NOSLIP`).
    fn wall_jacobian(&self, side: usize, e: &EdgeGeo) -> Blk {
        let refl = reflection_f(&e.nx, &e.nr);
        match self.visc.as_ref().and_then(|g| g.jmin_cols.as_ref()) {
            Some(m) if side == 2 => {
                let noslip: [f32; 16] = std::array::from_fn(|k| match k {
                    5 | 10 => -1.0,
                    _ => 0.0,
                });
                Blk(std::array::from_fn(|k| match &refl.0[k] {
                    E::A(a) => E::A(select(m, noslip[k], a)),
                    e => e.clone(),
                }))
            }
            _ => refl,
        }
    }
}

impl Blk {
    /// `self T` for `T = diag(1, -1, -1, 1)`, the Jacobian of the no-slip wall ghost (`NOSLIP`): columns 1 and 2 negated.
    fn noslip_cols(&self) -> Blk {
        Blk(std::array::from_fn(|k| match (&self.0[k], k % 4) {
            (E::A(a), 1 | 2) => E::A(-a),
            (E::O, 1 | 2) => unreachable!("no block of the scheme has a 1 in a momentum column off the diagonal"),
            (e, _) => e.clone(),
        }))
    }
}

impl LazySolver {
    /// [`LazySolver::base_wall_ghost`] in f32 (the left-hand side).
    fn base_wall_ghost_f(&self, w: &V4, nx: &F, nr: &F) -> V4 {
        if self.visc.is_some() {
            [w[0].clone(), -&w[1], -&w[2], w[3].clone()]
        } else {
            reflect_f(w, nx, nr)
        }
    }
}
