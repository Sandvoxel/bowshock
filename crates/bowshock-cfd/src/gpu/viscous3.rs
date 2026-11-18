// The 3D viscous terms in array ops. This file is `include!`d twice by `gpu/mod.rs` (once per precision, after `body3.rs`; it reuses `S`, `c`, `up`,
// `to_f64`, `A`, `Rec3`, `Vec3`, `rec3`, `select3`, `FaceGeo3`, `Geo3`, `Viscous`, `PRANDTL`, `PRANDTL_T`, `GAMMA` of the files before it).
//
// Every function mirrors the CPU routine of the same name in `crate::viscous3` (read its module header: the ghost of a face lies in the plane of the face,
// the reflected gradient across a symmetry plane, wall mask against update mask, the eddy viscosity of a no-slip face), expression by expression:
//
//   cell_pass     -> `viscous_cells3`    mu(T) and the Green-Gauss gradient of (u, v, w, T) of every cell
//   face_pass     -> `viscous_fluxes3`   the viscous flux Fv.n of every face of the three families
//   add_viscous_residual -> `viscous_residual3`
//   diffusion_rates      -> `diffusion_rates3`
//   lambda_pass   -> `lambda_pass3`
//   wall_tractions -> `wall_tractions3` (reduced on the device by wall class)
//
// The mass component of a viscous flux is zero: the fluxes here are four-component records (`Vec4`: the three momenta and the energy). The state of every
// kernel is the padded primitive record `w` (`[ni + 4, nj + 4, nk + 4]`, `pidx3`) that `assign_ghosts3` fills; the face data of the host (`face_dirs`,
// `face_cells`) is uploaded once as `VGeo3`.

use crate::sst::{IsoWall, ISO_GHOST_T_MIN};
use crate::viscous3::{face_cells, face_dirs, face_shape, SideKind, SymSides};

/// Four fields: the momentum and energy components of a viscous flux (the mass component is zero), or the quantities `(u, v, w, T)`.
pub type Vec4 = [A; 4];
/// The cell gradient `[u_x u_y u_z, v_x v_y v_z, w_x w_y w_z, T_x T_y T_z]` (`Grad3`) as twelve fields.
pub type Grad12 = [A; 12];

/// Face data of one family `dir` of the viscous kernels (`FaceDirs3` and `face_cells` of the host, in the face layout `[n0, n1, n2]` of the family).
pub(crate) struct VFace3 {
    /// The unit vector `e` from the lower to the upper side (zero where the face carries no flux).
    e: Vec3,
    /// `1 / |d|` (zero where the face carries no flux).
    inv_d: A,
    /// The face carries a viscous flux (`FaceDir3::d != 0`).
    flux: Array<bool>,
    /// `face_cells` is `Some`: the face counts in the Green-Gauss sum of its cells.
    fc: Array<bool>,
    /// A no-slip wall face (a fluid / solid face); read by the integration (S5b: the wall flux and the traction faces).
    #[allow(dead_code)]
    noslip: Array<bool>,
    /// The lower / upper side is not a cell: a domain ghost or the wall ghost. A ghost side carries the data of the fluid cell next to it.
    ghost: [Array<bool>; 2],
    /// The lower / upper side is the no-slip wall ghost `(rho, -u, -v, -w, p)` of the fluid cell.
    wall: [Array<bool>; 2],
}

/// The host-built geometry of the viscous kernels.
pub(crate) struct VGeo3 {
    dir: [VFace3; 3],
    sym: SymSides,
    v: Viscous,
    /// Isothermal wall condition (`None`: adiabatic): the ghost of the no-slip walls and their viscosity (`viscous3::wall_ghost_thermal3`, `face_mu_cells`).
    iso: Option<IsoWall>,
}

impl VGeo3 {
    /// `solid` is the per-cell update mask (solid + dead cells, the CPU `Solver3::solid`), `sym` the symmetry sides of the domain box.
    pub(crate) fn new(ctx: &Context, grid: &Grid3, solid: &[bool], sym: &SymSides, v: Viscous, iso: Option<IsoWall>) -> Result<VGeo3> {
        let dirs = face_dirs(grid, solid, sym);
        let family = |dir: usize| -> Result<VFace3> {
            let shape = face_shape(grid, dir);
            let nf = shape[0] * shape[1] * shape[2];
            let fd = dirs.family(dir);
            let (mut fc, mut ghost, mut wall) = (vec![false; nf], [vec![false; nf], vec![false; nf]], [vec![false; nf], vec![false; nf]]);
            for k in 0..shape[2] {
                for j in 0..shape[1] {
                    for i in 0..shape[0] {
                        let q = (k * shape[1] + j) * shape[0] + i;
                        if let Some(f) = face_cells(grid, solid, dir, i, j, k) {
                            fc[q] = true;
                            for s in 0..2 {
                                // the jet side of an exit face reads the jet cell's own data (state, `mu`, `mu_t`, gradient, scalars: `LazySolver3::jet_fix3`), not the
                                // fluid cell next to it as a wall or domain ghost does
                                ghost[s][q] = !matches!(f.kind[s], SideKind::Cell | SideKind::Jet);
                                wall[s][q] = f.kind[s] == SideKind::Wall;
                            }
                        }
                    }
                }
            }
            let mask = |v: &[bool]| Array::from_slice_on(ctx, v, shape);
            let e = |a: usize| up(ctx, &fd.iter().map(|f| f.e[a]).collect::<Vec<_>>(), shape);
            Ok(VFace3 {
                e: [e(0)?, e(1)?, e(2)?],
                inv_d: up(ctx, &fd.iter().map(|f| if f.d == 0.0 { 0.0 } else { 1.0 / f.d }).collect::<Vec<_>>(), shape)?,
                flux: mask(&fd.iter().map(|f| f.d != 0.0).collect::<Vec<_>>())?,
                fc: mask(&fc)?,
                noslip: mask(&fd.iter().map(|f| f.noslip).collect::<Vec<_>>())?,
                ghost: [mask(&ghost[0])?, mask(&ghost[1])?],
                wall: [mask(&wall[0])?, mask(&wall[1])?],
            })
        };
        Ok(VGeo3 { dir: [family(0)?, family(1)?, family(2)?], sym: *sym, v, iso })
    }
}

/// The result of the cell pass: the gradient record and the viscosity `mu(T)` of every cell.
pub(crate) struct VCell3 {
    grad: Grad12,
    mu: A,
}

/// `T = gamma p / rho`.
fn temperature3(w: &Rec3) -> A {
    c(GAMMA) * &w[4] / &w[0]
}

/// `(u, v, w, T)` of a primitive record.
fn phi3(w: &Rec3) -> Vec4 {
    [w[1].clone(), w[2].clone(), w[3].clone(), temperature3(w)]
}

/// Edge pad of a record of `[ni, nj, nk]` fields along `dir` by `before` / `after` cells.
fn pad_edge3<F: Fields<Elem = S>>(a: &F, dir: usize, before: usize, after: usize) -> F {
    match dir {
        0 => a.pad([(before, after)], PadMode::Edge),
        1 => a.pad([(0, 0), (before, after)], PadMode::Edge),
        _ => a.pad([(0, 0), (0, 0), (before, after)], PadMode::Edge),
    }
}

impl VGeo3 {
    /// The primitive states of the two sides of every face of `dir` from the padded record `w`: the lower and the upper cell of the face (a domain ghost layer
    /// at the boundary), a no-slip wall side replaced by the wall ghost of the fluid cell on its other side (`face_sides3`): `(rho, -u, -v, -w, p)`, or for an isothermal wall
    /// `rho = gamma p / T_g` with `T_g = max(2 T_w - T, ISO_GHOST_T_MIN T_w)` (`viscous3::wall_ghost_thermal3`).
    fn side_states3(&self, w: &Rec3, dir: usize, n: [usize; 3]) -> (Rec3, Rec3) {
        let mut line = w.clone();
        for a in (0..3).filter(|&a| a != dir) {
            line = line.slice(a, 2..n[a] + 2);
        }
        let (lo, hi) = (line.slice(dir, 1..n[dir] + 2), line.slice(dir, 2..n[dir] + 3));
        let ghost = |s: &Rec3| -> Rec3 {
            let rho = match self.iso {
                None => s[0].clone(),
                Some(i) => {
                    let t_g = (c(2.0 * i.t_w) - temperature3(s)).maximum(c(ISO_GHOST_T_MIN * i.t_w));
                    c(GAMMA) * &s[4] / &t_g
                }
            };
            [rho, -&s[1], -&s[2], -&s[3], s[4].clone()]
        };
        let f = &self.dir[dir];
        (select3(&f.wall[0], &ghost(&hi), &lo), select3(&f.wall[1], &ghost(&lo), &hi))
    }

    /// Cell data `a` (`[ni, nj, nk]` fields) on the two sides of every face of `dir`: the cell below and above; a ghost side (domain ghost or wall) carries the data of
    /// the fluid cell next to it.
    fn cell_sides3<F: Fields<Elem = S>>(&self, a: &F, dir: usize) -> (F, F) {
        let f = &self.dir[dir];
        let (lo, hi) = (pad_edge3(a, dir, 1, 0), pad_edge3(a, dir, 0, 1));
        (hi.zip(&lo, |h, l| select(&f.ghost[0], h, l)), lo.zip(&hi, |l, h| select(&f.ghost[1], l, h)))
    }
}

/// The gradient record of the mirror image of the field about the plane of unit normal `n` (`viscous3::reflect_grad`): `R G R` for the velocity, `R grad T`.
fn reflect_grad3(g: &[A], n: &Vec3) -> Vec<A> {
    let r = |a: usize, b: usize| -> A { (if a == b { c(1.0) - c(2.0) * &n[a] * &n[b] } else { -(c(2.0) * &n[a] * &n[b]) }).clone() };
    let rm: Vec<A> = (0..9).map(|q| r(q / 3, q % 3)).collect();
    let mut out = Vec::with_capacity(12);
    for a in 0..3 {
        for b in 0..3 {
            let mut acc: Option<A> = None;
            for cc in 0..3 {
                for d in 0..3 {
                    let term = &rm[3 * a + cc] * &g[3 * cc + d] * &rm[3 * d + b];
                    acc = Some(match acc {
                        None => term,
                        Some(x) => x + term,
                    });
                }
            }
            out.push(acc.expect("nine terms"));
        }
    }
    for a in 0..3 {
        out.push(&rm[3 * a] * &g[9] + &rm[3 * a + 1] * &g[10] + &rm[3 * a + 2] * &g[11]);
    }
    out
}

impl LazySolver3 {
    /// The `Staging` cache applied to a record: evaluated, or left lazy for `Staging::Fusion`.
    fn staged3<F: Fields<Elem = S>>(&self, x: F) -> F {
        if self.staging == Staging::Fusion {
            x
        } else {
            x.cache()
        }
    }

    /// Switches the viscous terms on: the host geometry of the face passes (`grid`, `sym` the symmetry sides of the domain box) and the gas parameters.
    pub fn set_viscous3(&mut self, grid: &Grid3, v: Viscous, sym: &SymSides, iso: Option<IsoWall>) -> Result<()> {
        self.compiled = None;
        let solid: Vec<bool> = grid.part.iter().map(|&p| p != crate::grid3::PART_FLUID).collect();
        self.visc = Some(VGeo3::new(&self.ctx, grid, &solid, sym, v, iso)?);
        Ok(())
    }

    /// Pass 1 (`viscous3::cell_pass`): the viscosity `mu(T)` and the Green-Gauss gradient `g = (1 / V) sum_faces phi_f S_f`, `phi_f = (phi_c + phi_neighbour) / 2`,
    /// of `(u, v, w, T)` of every fluid cell (zero in solid cells). `p` is the unpadded primitive record, `w` the padded one.
    fn viscous_cells3(&self, vg: &VGeo3, p: &Rec3, w: &Rec3) -> VCell3 {
        let n = [self.ni, self.nj, self.nk];
        let g = &self.geo;
        let v = vg.v;
        let t = temperature3(p);
        let mu = (&t * t.sqrt()) * c(1.0 + v.sutherland) / (&t + c(v.sutherland)) / c(v.re_c);
        let pc = phi3(p);
        let mut acc: Vec<Option<A>> = vec![None; 12];
        for dir in 0..3 {
            let (f, fg) = (&vg.dir[dir], &g.f[dir]);
            let (l, r) = vg.side_states3(w, dir, n);
            let (phl, phr) = (phi3(&l), phi3(&r));
            let (pc_lo, pc_hi) = (pad_edge3(&pc, dir, 1, 0), pad_edge3(&pc, dir, 0, 1));
            let na: Vec3 = std::array::from_fn(|a| &fg.n[a] * &fg.area);
            for m in 0..4 {
                // the face value of the neighbour average, seen from the cell below (through its upper face) and from the cell above (through its lower face)
                let from_below = select(&f.fc, c(0.5) * (&pc_lo[m] + &phr[m]), c(0.0));
                let from_above = select(&f.fc, c(0.5) * (&pc_hi[m] + &phl[m]), c(0.0));
                for a in 0..3 {
                    let term = (&from_below * &na[a]).slice(dir, 1..n[dir] + 1) - (&from_above * &na[a]).slice(dir, 0..n[dir]);
                    let q = 3 * m + a;
                    acc[q] = Some(match acc[q].take() {
                        None => term,
                        Some(x) => x + term,
                    });
                }
            }
        }
        let grad: Grad12 = std::array::from_fn(|q| {
            let x = acc[q].take().expect("six faces") * &g.inv_v;
            match &g.fluid {
                Some(fluid) => select(fluid, x, c(0.0)),
                None => x,
            }
        });
        VCell3 { grad: self.staged3(grad), mu: self.staged3(mu) }
    }

    /// Face viscosity and eddy viscosity (`viscous3::face_mu3`): the mean of the two sides, a ghost side taking its cell's values, the eddy viscosity of a wall
    /// ghost `2 mu_t,w - mu_t` (`mu_t,w` zero for a smooth wall: `-mu_t`, a face value 0) and, across an isothermal wall, the laminar viscosity `2 mu(T_w) - mu` (a face value `mu(T_w)`).
    /// `cl`, `cr`: the cell sides of `[mu, mu_t, mu_t,w]`.
    fn face_mu3(&self, vg: &VGeo3, dir: usize, cl: &[A; 3], cr: &[A; 3]) -> (A, A) {
        let f = &vg.dir[dir];
        let mut_l = select(&f.wall[0], c(2.0) * &cl[2] - &cl[1], &cl[1]);
        let mut_r = select(&f.wall[1], c(2.0) * &cr[2] - &cr[1], &cr[1]);
        let mu_f = match vg.iso {
            None => c(0.5) * (&cl[0] + &cr[0]),
            Some(i) => {
                let mu_l = select(&f.wall[0], c(2.0 * i.mu_w) - &cl[0], &cl[0]);
                let mu_r = select(&f.wall[1], c(2.0 * i.mu_w) - &cr[0], &cr[0]);
                c(0.5) * (&mu_l + &mu_r)
            }
        };
        (mu_f, c(0.5) * (&mut_l + &mut_r))
    }

    /// Pass 2 (`viscous3::face_pass`): the viscous flux `Fv.n` of every face of the three families, as `[Fv_x, Fv_y, Fv_z, Fv_E]` (the mass component is zero),
    /// zero where the face carries no flux. `mu_t` is the eddy viscosity per cell, `wall_mut` the wall eddy viscosity `mu_t,w` per cell (`None`: smooth walls).
    fn viscous_fluxes3(&self, vg: &VGeo3, w: &Rec3, cell: &VCell3, mu_t: &A, wall_mut: Option<&A>) -> [Vec4; 3] {
        let n = [self.ni, self.nj, self.nk];
        let g = &self.geo;
        let zero = &cell.mu * c(0.0);
        let scal: [A; 3] = [cell.mu.clone(), mu_t.clone(), wall_mut.cloned().unwrap_or(zero)];
        std::array::from_fn(|dir| {
            let (f, fg) = (&vg.dir[dir], &g.f[dir]);
            let (l, r) = vg.side_states3(w, dir, n);
            let (sl, sr) = vg.cell_sides3(&scal, dir);
            let (gl, gr) = vg.cell_sides3(&cell.grad, dir);
            let (mut gl, mut gr): (Vec<A>, Vec<A>) = (gl.to_vec(), gr.to_vec());
            // a ghost across a symmetry plane carries the reflected gradient of its cell
            if vg.sym[2 * dir] {
                let nl: Vec3 = std::array::from_fn(|a| fg.n[a].slice(dir, 0..1));
                let slab: Vec<A> = gl.iter().map(|x| x.slice(dir, 0..1)).collect();
                for (q, x) in reflect_grad3(&slab, &nl).into_iter().enumerate() {
                    gl[q].slice_mut(dir, 0..1).assign(x).expect("symmetry slab assign: shapes fixed by the grid");
                }
            }
            if vg.sym[2 * dir + 1] {
                let nu: Vec3 = std::array::from_fn(|a| fg.n[a].slice(dir, n[dir]..n[dir] + 1));
                let slab: Vec<A> = gr.iter().map(|x| x.slice(dir, n[dir]..n[dir] + 1)).collect();
                for (q, x) in reflect_grad3(&slab, &nu).into_iter().enumerate() {
                    gr[q].slice_mut(dir, n[dir]..n[dir] + 1).assign(x).expect("symmetry slab assign: shapes fixed by the grid");
                }
            }
            let (pl, pr) = (phi3(&l), phi3(&r));
            // the corrected face gradient `gbar + ((phi_R - phi_L) / d - gbar . e) e` of (u, v, w, T)
            let mut gf: Vec<A> = Vec::with_capacity(12);
            for m in 0..4 {
                let gb: [A; 3] = std::array::from_fn(|a| c(0.5) * (&gl[3 * m + a] + &gr[3 * m + a]));
                let corr = (&pr[m] - &pl[m]) * &f.inv_d - (&gb[0] * &f.e[0] + &gb[1] * &f.e[1] + &gb[2] * &f.e[2]);
                for a in 0..3 {
                    gf.push(&gb[a] + &corr * &f.e[a]);
                }
            }
            let (mu_f, mut_f) = self.face_mu3(vg, dir, &[sl[0].clone(), sl[1].clone(), sl[2].clone()], &[sr[0].clone(), sr[1].clone(), sr[2].clone()]);
            let uf: [A; 3] = std::array::from_fn(|a| c(0.5) * (&l[1 + a] + &r[1 + a]));
            let div = &gf[0] + &gf[4] + &gf[8];
            let mue = &mu_f + &mut_f;
            let kappa = (&mu_f / c(PRANDTL) + &mut_f / c(PRANDTL_T)) / c(GAMMA - 1.0);
            let t: [A; 3] = std::array::from_fn(|a| {
                let tau = |b: usize| {
                    let sym = &gf[3 * a + b] + &gf[3 * b + a];
                    &mue * if a == b { sym - c(2.0 / 3.0) * &div } else { sym }
                };
                tau(0) * &fg.n[0] + tau(1) * &fg.n[1] + tau(2) * &fg.n[2]
            });
            let heat = &kappa * (&gf[9] * &fg.n[0] + &gf[10] * &fg.n[1] + &gf[11] * &fg.n[2]);
            let energy = &uf[0] * &t[0] + &uf[1] * &t[1] + &uf[2] * &t[2] + heat;
            let [t0, t1, t2] = t;
            let zero = |x: A| select(&f.flux, x, c(0.0));
            [zero(t0), zero(t1), zero(t2), zero(energy)]
        })
    }

    /// The viscous part of the residual `dU/dt += (1 / V) sum_faces +-(Fv.n) A` of the momentum and energy (`viscous3::add_viscous_residual`; zero in solid cells
    /// because `1 / V` is zero there).
    fn viscous_residual3(&self, fv: &[Vec4; 3]) -> Vec4 {
        let n = [self.ni, self.nj, self.nk];
        let g = &self.geo;
        std::array::from_fn(|q| {
            let mut acc: Option<A> = None;
            for dir in 0..3 {
                let area = &g.f[dir].area;
                let flux = &fv[dir][q] * area;
                let term = flux.slice(dir, 1..n[dir] + 1) - flux.slice(dir, 0..n[dir]);
                acc = Some(match acc {
                    None => term,
                    Some(x) => x + term,
                });
            }
            acc.expect("three directions") * &g.inv_v
        })
    }

    /// The face flux of a viscous run, `F - Fv` componentwise (the mass component is the inviscid one); used by the integration (S5b).
    #[allow(dead_code)]
    fn total_flux3(f: &Rec3, fv: &Vec4) -> Rec3 {
        [f[0].clone(), &f[1] - &fv[0], &f[2] - &fv[1], &f[3] - &fv[2], &f[4] - &fv[3]]
    }

    /// `2 lv A_f^2 / V` per direction, `lv = gamma (mu / PRANDTL + mu_t / PRANDTL_T) / rho`, `A_f` the mean area of the two faces of the direction
    /// (`viscous3::diffusion_rates`).
    fn diffusion_rates3(&self, cell: &VCell3, mu_t: &A, rho: &A) -> [A; 3] {
        let n = [self.ni, self.nj, self.nk];
        let g = &self.geo;
        let lv = c(GAMMA) * (&cell.mu / c(PRANDTL) + mu_t / c(PRANDTL_T)) / rho;
        std::array::from_fn(|dir| {
            let area = c(0.5) * (g.f[dir].area.slice(dir, 0..n[dir]) + g.f[dir].area.slice(dir, 1..n[dir] + 1));
            c(2.0) * &lv * &area * &area / &g.vol
        })
    }

    /// The diffusive spectral radius `lam = gamma (mu_f / PRANDTL + mu_t,f / PRANDTL_T) / (rho_f |d|)` of every face of the three families, zero where the face
    /// carries no flux (`viscous3::lambda_pass`).
    fn lambda_pass3(&self, vg: &VGeo3, w: &Rec3, cell: &VCell3, mu_t: &A, wall_mut: Option<&A>) -> [A; 3] {
        let n = [self.ni, self.nj, self.nk];
        let zero = &cell.mu * c(0.0);
        let scal: [A; 3] = [cell.mu.clone(), mu_t.clone(), wall_mut.cloned().unwrap_or(zero)];
        std::array::from_fn(|dir| {
            let f = &vg.dir[dir];
            let (l, r) = vg.side_states3(w, dir, n);
            let (sl, sr) = vg.cell_sides3(&scal, dir);
            let (mu_f, mut_f) = self.face_mu3(vg, dir, &sl, &sr);
            let rho_f = c(0.5) * (&l[0] + &r[0]);
            let lam = c(GAMMA) * (&mu_f / c(PRANDTL) + &mut_f / c(PRANDTL_T)) / &rho_f * &f.inv_d;
            select(&f.flux, lam, c(0.0))
        })
    }
}

/// Device arrays of the viscous wall tractions of one face family: the faces (flat index `q` of the family) with their weight `-sigma area` (so that the force on
/// the wall is `Fv.n[1..4] * weight`), the abscissa of the centroid and the class masks.
struct TractionFam3 {
    idx: Array<u32>,
    wa: A,
    cx: A,
    class: Vec<Array<bool>>,
}

/// The no-slip faces of the viscous force reduction (see [`LazySolver3::set_traction_faces`]).
struct TractionGeo3 {
    fam: [Option<TractionFam3>; 3],
    n_classes: usize,
}

impl LazySolver3 {
    /// Uploads the no-slip faces of the viscous force integration (`crate::gpu::TractionFace3`, as `viscous3::wall_tractions` finds them) for `n_classes` classes.
    pub fn set_traction_faces(&mut self, faces: &[crate::gpu::TractionFace3], n_classes: usize) -> Result<()> {
        let ctx = &self.ctx;
        let mut fam: [Option<TractionFam3>; 3] = [None, None, None];
        for (dir, slot) in fam.iter_mut().enumerate() {
            let list: Vec<&crate::gpu::TractionFace3> = faces.iter().filter(|f| f.dir == dir).collect();
            if list.is_empty() {
                continue;
            }
            let n = list.len();
            let idx: Vec<u32> = list.iter().map(|f| f.q as u32).collect();
            let class = (0..n_classes)
                .map(|cl| Array::from_slice_on(ctx, &list.iter().map(|f| f.class == cl).collect::<Vec<_>>(), [n]))
                .collect::<Result<Vec<_>>>()?;
            *slot = Some(TractionFam3 {
                idx: Array::from_slice_on(ctx, &idx, [n])?,
                wa: up(ctx, &list.iter().map(|f| -f.sigma * f.area).collect::<Vec<_>>(), [n])?,
                cx: up(ctx, &list.iter().map(|f| f.cx).collect::<Vec<_>>(), [n])?,
                class,
            });
        }
        self.traction = Some(TractionGeo3 { fam, n_classes });
        Ok(())
    }

    /// The viscous force on the body by class, `[Fx, Fz, Mz]` as the pressure forces of `wall_forces` (`Mz = sum x Fz`), reduced on the device from the face
    /// fluxes `fv` of [`LazySolver3::viscous_fluxes3`]: the force on the wall is `tau = -sigma Fv.n[1..4]` (`viscous3::wall_tractions`). One download of
    /// `3 n_classes` numbers.
    fn wall_tractions3(&self, fv: &[Vec4; 3]) -> Result<Vec<[f64; 3]>> {
        let tg = self.traction.as_ref().ok_or_else(|| luma::LumaError::Unsupported("traction faces are not set".into()))?;
        let mut total: Option<A> = None;
        for (dir, slot) in tg.fam.iter().enumerate() {
            let Some(f) = slot else { continue };
            let fx = fv[dir][0].take(&f.idx) * &f.wa;
            let fz = fv[dir][2].take(&f.idx) * &f.wa;
            let mz = &fz * &f.cx;
            let sums: Vec<A> = f.class.iter().flat_map(|m| [fx.sum_where(m).reshape([1]), fz.sum_where(m).reshape([1]), mz.sum_where(m).reshape([1])]).collect();
            if sums.is_empty() {
                continue;
            }
            let s = luma::concat_slice(0, &sums);
            total = Some(match total {
                None => s,
                Some(t) => t + s,
            });
        }
        let Some(total) = total else { return Ok(vec![[0.0; 3]; tg.n_classes]) };
        Ok(to_f64(&total)?.chunks(3).map(|t| [t[0], t[1], t[2]]).collect())
    }

    /// Runs the viscous kernels on a padded primitive state of the host (`w`: `(ni + 4) (nj + 4) (nk + 4)` states in the layout of `pidx3`; `mu_t` per cell, the
    /// wall eddy viscosity per cell `wall_mut` or `None`) and returns everything they produce, downloaded: the kernel-level comparison with the CPU `viscous3`.
    pub fn viscous_probe3(&self, w_host: &[crate::gas3::Prim3], mu_t: &[f64], wall_mut: Option<&[f64]>) -> Result<crate::gpu::VisProbe> {
        let vg = self.visc.as_ref().ok_or_else(|| luma::LumaError::Unsupported("viscous terms are not set".into()))?;
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let shape = [ni + 4, nj + 4, nk + 4];
        let comp = |q: usize| up(&self.ctx, &w_host.iter().map(|x| x[q]).collect::<Vec<_>>(), shape);
        let w: Rec3 = [comp(0)?, comp(1)?, comp(2)?, comp(3)?, comp(4)?];
        let p: Rec3 = rec3(|q| w[q].slice(0, 2..ni + 2).slice(1, 2..nj + 2).slice(2, 2..nk + 2));
        let cell_shape = [ni, nj, nk];
        let mu_t_a = up(&self.ctx, mu_t, cell_shape)?;
        let wall_a = wall_mut.map(|m| up(&self.ctx, m, cell_shape)).transpose()?;
        let cell = self.viscous_cells3(vg, &p, &w);
        let fv = self.viscous_fluxes3(vg, &w, &cell, &mu_t_a, wall_a.as_ref());
        let res = self.viscous_residual3(&fv);
        let lam = self.lambda_pass3(vg, &w, &cell, &mu_t_a, wall_a.as_ref());
        let dr = self.diffusion_rates3(&cell, &mu_t_a, &p[0]);
        let tractions = if self.traction.is_some() { self.wall_tractions3(&fv)? } else { Vec::new() };
        let down = |a: &[A]| -> Result<Vec<Vec<f64>>> { a.iter().map(to_f64).collect() };
        let rows = |v: &[Vec<f64>]| -> Vec<Vec<f64>> { (0..v[0].len()).map(|i| v.iter().map(|c| c[i]).collect()).collect() };
        let grad = rows(&down(&cell.grad)?).into_iter().map(|r| std::array::from_fn(|q| r[q])).collect();
        let fv_rows = |f: &Vec4| -> Result<Vec<[f64; 5]>> { Ok(rows(&down(f)?).into_iter().map(|r| [0.0, r[0], r[1], r[2], r[3]]).collect()) };
        let res_rows = rows(&down(&res)?).into_iter().map(|r| [0.0, r[0], r[1], r[2], r[3]]).collect();
        Ok(crate::gpu::VisProbe {
            grad,
            mu: to_f64(&cell.mu)?,
            fv: [fv_rows(&fv[0])?, fv_rows(&fv[1])?, fv_rows(&fv[2])?],
            lambda: [to_f64(&lam[0])?, to_f64(&lam[1])?, to_f64(&lam[2])?],
            dr: rows(&down(&dr)?).into_iter().map(|r| [r[0], r[1], r[2]]).collect(),
            residual: res_rows,
            tractions,
        })
    }
}

impl LazySolver3 {
    /// One evaluation of the viscous residual of the current state (laminar: `mu_t` = 0; the cell pass, the face fluxes of the three families, the residual and the
    /// diffusive spectral radius of the faces), the padded primitives evaluated before the profile starts: the launches and GPU time of the kernels alone, and where
    /// fusion cut the expressions. Needs [`LazySolver3::set_viscous3`] and a state.
    pub fn profile_viscous3(&mut self) -> Result<luma::Profile> {
        let ctx = self.ctx.clone();
        let run = |s: &LazySolver3| -> Result<()> {
            let vg = s.visc.as_ref().ok_or_else(|| luma::LumaError::Unsupported("viscous terms are not set".into()))?;
            let u = unpack3(s.state.as_ref().ok_or_else(no_state)?);
            let (p, w) = s.padded3(&u);
            let cell = s.viscous_cells3(vg, &p, &w);
            let mu_t = &cell.mu * c(0.0);
            let fv = s.viscous_fluxes3(vg, &w, &cell, &mu_t, None);
            let res = s.viscous_residual3(&fv);
            let lam = s.lambda_pass3(vg, &w, &cell, &mu_t, None);
            let mut all: Vec<&A> = res.iter().collect();
            all.extend(lam.iter());
            luma::eval_all(&all)?;
            s.ctx.sync()
        };
        run(self)?;
        let (out, profile) = ctx.profile(|| run(self))?;
        out?;
        Ok(profile)
    }
}
