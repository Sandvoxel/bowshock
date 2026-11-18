//! Viscous terms of the 3D compressible Navier-Stokes equations on a [`Grid3`] (laminar; the eddy viscosity is an input array, zero until a
//! turbulence model is attached). The 3D counterpart of [`crate::viscous`], in Cartesian components: no hoop stress and no `1 / r`.
//!
//! Units as in 2D: `rho_inf = 1`, `c_inf = 1`, `p_inf = 1 / gamma`, `T = gamma p / rho` (`T_inf = 1`), lengths in nose diameters.
//!
//! ```text
//!   mu(T)  = (1 / re_c) T^1.5 (1 + S) / (T + S)                       S = sutherland (Viscous::mu)
//!   kappa  = (mu / PRANDTL + mu_t / PRANDTL_T) / (gamma - 1)          q = -kappa grad T        (c_p = 1 / (gamma - 1))
//!   tau_ij = mue (d_j u_i + d_i u_j - 2/3 div delta_ij)               div = d_x u + d_y v + d_z w,   mue = mu + mu_t
//!   Fv.n   = [0, tau_xj n_j, tau_yj n_j, tau_zj n_j, u_i tau_ij n_j + kappa d_j T n_j]
//!   dU/dt  = ... + (1 / V) sum_faces +-(Fv.n) A            (the residual uses the face flux F - Fv; `add_viscous_residual`)
//! ```
//!
//! `u = (u, v, w)` is Cartesian (`u` axial, as [`crate::gas3`]), the gradient record of a cell is `[u_x u_y u_z, v_x v_y v_z, w_x w_y w_z, T_x T_y T_z]`
//! ([`Grad3`], `grad[3 m + d] = d phi_m / d x_d`). Three data-parallel passes, called after the inviscid face fluxes:
//!
//! 1. [`cell_pass`]: `mu(T)` and the Green-Gauss gradient of `(u, v, w, T)` of every fluid cell, `g = (1 / V) sum_faces phi_f S_f` with
//!    `S_f = +-A n` (outward) and `phi_f = (phi_c + phi_neighbour) / 2`;
//! 2. [`face_pass`]: the face gradient `g_f = gbar + ((phi_R - phi_L) / |d| - gbar.e) e` (`d` = distance vector of the two sides, `e = d / |d|`, `gbar`
//!    the mean of the two side gradients: a compact normal derivative with the averaged tangential part) and the viscous flux of every i-, j- and
//!    k-face; the geometry `(e, |d|)` of every face comes precomputed from [`face_dirs`];
//! 3. [`add_viscous_residual`] (and [`diffusion_rate`], [`lambda_pass`] for the time step and the implicit operator).
//!
//! What is a wall, a boundary and a ghost (the single classification of every kernel here, [`face_cells`]):
//!
//! * `solid[c]` is the per-cell solid mask (`Solver3::solid`; every cell of a body, a fin or an appendage). The state `w` of a solid cell is never read.
//!   A face between a fluid and a solid cell is a **no-slip wall**, adiabatic by default: the solid side is the ghost `(rho, -u, -v, -w, p)` of the fluid cell
//!   ([`wall_ghost3`]); with an isothermal wall (`iso: Option<IsoWall>`, [`crate::sst::WallThermal`]) the ghost of these kernels has the temperature `T_g = 2 T_w - T` at the same
//!   pressure ([`wall_ghost_thermal3`]: the face value of `T` is `T_w` and the compact gradient is the heat-flux gradient of the wall) and the face viscosity is `mu(T_w)`
//!   ([`face_mu_cells`]). A face between two solid cells carries no flux, nor does a face with `area == 0` (the cells of an axis run have degenerate faces).
//! * A face on the domain boundary (`i = 0, ni`, `j = 0, nj`, `k = 0, nk`) has the first ghost layer of the padded array `w` ([`pidx3`]) on its outer side:
//!   whatever the solver put there (far field, symmetry reflection, axis map). A face on the boundary whose inner cell is solid carries no flux.
//! * A ghost sits at the mirror image of the centre of its fluid cell **in the plane of the face** (through its centroid, normal `Face3::n`; the 2D module
//!   mirrors in the face centre, which is the same on an orthogonal cell). The compact gradient then acts along the face normal even on a skewed or curved
//!   cell: the wall carries no heat flux through the adiabatic wall ghost (the line to a point-mirrored ghost is tilted by the skew angle of the cell), and on a wedge of opening angle `dtheta` the hoop
//!   terms, which come from the compact gradient across the two symmetry faces and have a weight `1 / (r dtheta)`, do not pick up an `O(dtheta)` tilt.
//!   A ghost carries the viscosity and eddy viscosity of its cell and its gradient, except across a **symmetry plane** ([`SymSides`], set by the solver for the
//!   sides of the domain box whose ghost layer is the reflection of the cells): there it carries the gradient of the reflected field `R G R`
//!   ([`reflect_grad`]), so the face tensor is the average of the tensors of the cell and its mirror image and the shear `n.G.t` of a curved flow cut by the plane
//!   cancels (the unreflected gradient leaves an `O(dtheta)` shear there, which the weight `1 / (r dtheta)` turns into an `O(1)` error of the residual).
//!   Face viscosity and eddy viscosity are the mean of the two sides ([`face_mu3`]); across a no-slip face the ghost eddy viscosity is `2 mu_t,w - mu_t`
//!   with the wall eddy viscosity `mu_t,w = wall[cell][2]` of the turbulence wall treatment (`-mu_t`, i.e. a face value 0, when `wall` is empty).
//!
//! # Differences from the 2D module and DESIGN3D section C (read this before integrating or porting)
//!
//! 1. **Ghost position**: the mirror image of the cell centre in the *plane of the face* (through its centroid, normal `Face3::n`), not in the face centre.
//!    The compact gradient `(phi_R - phi_L) / d` then acts along the face normal (`FaceDir3::e` is the normal on every wall and boundary face). Equal to the 2D
//!    rule on an orthogonal cell. A point mirror tilts `e` by half the opening angle on a wedge `k` face, which the weight `1 / (r dtheta)` of the hoop terms
//!    turns into an `O(1)` error of the residual (measured: radial-momentum error 5 against the analytic operator, growing like `1 / h`, independent of `dtheta`).
//! 2. **`SymSides`** (`[i0, i1, j0, j1, k0, k1]`, given to [`face_dirs`] by the solver): the sides of the domain box whose ghost layer is the reflection
//!    `gas3::reflect3` of the cells (wedge, sector and half-domain `k` planes). A ghost across such a side carries the *reflected* gradient `R G R`
//!    ([`reflect_grad`]) instead of the gradient of its cell; the face tensor is then the mean of the tensors of the cell and its mirror image and carries no
//!    shear `n.G.t` (the unreflected gradient leaves an `O(dtheta)` shear, again `O(1)` after the `1 / (r dtheta)` weight). Far-field, outflow and axis ghosts
//!    keep the gradient of their cell; a no-slip wall ghost `(rho, -u, -v, -w, p)` too (the odd field has the same gradient).
//! 3. **`wall: &[[f64; 3]]`** replaces the 2D `rough` argument of `face_mu`: `[k_w, omega_w, mu_t,w]` per cell with a no-slip face, produced by the
//!    turbulence wall treatment for every wall cell (smooth walls too, `turbulence3`); an empty slice is a smooth wall with `mu_t,w = 0`. A cell with two wall
//!    faces (a fin-body junction) has one state for both.
//! 4. **`diffusion_rate`** is `2 lv A_f^2 / V` (mean face area of the direction, cell volume): the 2D `2 lv len^2 / A` with face length -> face area and cell
//!    area -> volume, the partner of the convective `(|u.n| + c) A_f` of `Solver3::compute_dt`. [`face_lambda3`] is the 2D `face_lambdas`.
//! 5. **Types**: `crate::solver::{Viscous, PRANDTL, PRANDTL_T}` and `Viscous::mu` are reused, not moved (only `re_c`, `sutherland` are read); the solid mask is the
//!    per-cell `Solver3::solid` (full length), never `Grid3::solid` boxes; the Fv.n arrays have the layout and 5 components of `Solver3::{fi, fj, fk}` (mass
//!    component 0), so the residual is `F - Fv` componentwise.
//! 6. **Grid note**: `grid3::revolve` has flat faces, so a wedge wall lies `r (1 - cos(dtheta / 2))` inside the revolved surface (0.14 % of `r` at 6 degrees);
//!    the parity fixture scales the node radii by `1 / cos(dtheta / 2)` ("matched") to compare against the 2D grid. And the hoop terms need `Grid3::xc` to be
//!    the mean of the 8 nodes (consistent with the face centroids): sampling a wedge at other points amplifies like `h^2 / dtheta`.
//! 7. **Wall forces**: [`wall_tractions`] reads the viscous force on the body from the very flux `Fv.n` of the no-slip faces the mean-flow equations see (face viscosity
//!    `mu + mu_t,w` of the wall treatment), so the friction drag is the sum of those fluxes and there is no second shear formula; the wall-function value
//!    `rho u_tau^2` (`turbulence3::wall_shear_vector`) is carried exactly as the tangential part of that flux.
//!
//! Plain functions of their inputs; every pass writes the whole output arrays (zero where there is no flux / no fluid) and keeps no state. The parameters
//! are the 2D ones (`crate::sst::Viscous`: only `re_c`, `sutherland` are read, `wall_i_start` is not used in 3D).
//!
//! Wedge parity (`parity` tests): a one-cell wedge `grid3::revolve(g2, 1, [0, dtheta])` with symmetry planes in `k` reproduces the viscous residual of
//! `viscous.rs` including its hoop-stress source. The wedge-angle error alone (against the same wedge at 0.05 degrees) falls like `dtheta^2` (ratios 4.0 / 9.0 /
//! 4.0 for 12 -> 6 -> 2 -> 1 degrees) and is `<= 5e-4` (relative `L2` of the cells away from the axis and the domain boundary) at 2 degrees; what is left against
//! the 2D residual is the difference of the two discretisations (area-weighted 3D against planar 2D Green-Gauss gradients, and the ghost of a no-slip wall: the plane
//! mirror here, so that the compact gradient is wall-normal and an adiabatic wall has no heat flux on a sheared grid, the point image of the cell centre in the face
//! centre in 2D), second order in the interior and `O(h / r)` in the first cells beside the axis and `O(1)` in the wall cells of a sheared grid (the nose of the G3
//! body: 18.8 degrees, the same on every refinement; `tests/viscous3.rs` `wedge_residual_of_the_2d_state`: the viscous part of the residual of the converged 2D state
//! differs by `0.05` in the radial momentum of those cells at 0.5, 2 and 8 degrees and on two grids). The `report` test prints the table.

use crate::gas3::{Cons3, Prim3, GAMMA};
use crate::grid3::{Face3, Grid3, Part, PART_DEAD, PART_FLUID};
use crate::solver3::pidx3;
use crate::sst::{IsoWall, Viscous, PRANDTL, PRANDTL_T};
use rayon::prelude::*;

/// Cell gradient `[u_x u_y u_z, v_x v_y v_z, w_x w_y w_z, T_x T_y T_z]`: `grad[3 m + d] = d phi_m / d x_d` of `phi = (u, v, w, T)`.
pub type Grad3 = [f64; 12];

/// Static temperature `gamma p / rho` of a primitive state.
#[inline]
pub fn temperature3(w: &Prim3) -> f64 {
    GAMMA * w[4] / w[0]
}

/// Ghost state across a no-slip adiabatic wall: `(rho, -u, -v, -w, p)` (zero normal pressure and temperature gradient).
#[inline]
pub fn wall_ghost3(w: &Prim3) -> Prim3 {
    [w[0], -w[1], -w[2], -w[3], w[4]]
}

/// Ghost state of the *viscous* kernels across a no-slip wall: the adiabatic [`wall_ghost3`] for `iso == None`; across an isothermal wall `(rho_g, -u, -v, -w, p)` with the
/// ghost temperature `T_g = 2 T_w - T` ([`IsoWall::ghost_t`]) at the same pressure, `rho_g = gamma p / T_g`, so that the face value of `T` is `T_w` and the compact
/// gradient `(T_g - T) / d` is the heat-flux gradient of the wall. (The inviscid fluxes and the implicit Jacobian keep the adiabatic ghost: the wall pressure is what they need.)
#[inline]
pub fn wall_ghost_thermal3(w: &Prim3, iso: Option<IsoWall>) -> Prim3 {
    match iso {
        None => wall_ghost3(w),
        Some(i) => [GAMMA * w[4] / i.ghost_t(temperature3(w)), -w[1], -w[2], -w[3], w[4]],
    }
}

/// `(u, v, w, T)` of a primitive state.
#[inline]
pub fn phi3(w: &Prim3) -> [f64; 4] {
    [w[1], w[2], w[3], temperature3(w)]
}

/// Number of faces per index of a family: `[ni + 1, nj, nk]` (i-faces), `[ni, nj + 1, nk]`, `[ni, nj, nk + 1]`; the faces are stored `(k * n1 + j) * n0 + i`.
#[inline]
pub fn face_shape(grid: &Grid3, dir: usize) -> [usize; 3] {
    [grid.ni + (dir == 0) as usize, grid.nj + (dir == 1) as usize, grid.nk + (dir == 2) as usize]
}

/// Index and geometry of face `(i, j, k)` of the family `dir` (0: i-faces, 1: j-faces, 2: k-faces).
#[inline]
pub fn face_of(grid: &Grid3, dir: usize, i: usize, j: usize, k: usize) -> (usize, &Face3) {
    let [n0, n1, _] = face_shape(grid, dir);
    let q = (k * n1 + j) * n0 + i;
    (
        q,
        match dir {
            0 => &grid.iface[q],
            1 => &grid.jface[q],
            _ => &grid.kface[q],
        },
    )
}

/// What a side of a face is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SideKind {
    /// A fluid cell.
    Cell,
    /// A ghost of the padded primitive array `w` across a domain boundary.
    Domain,
    /// The solid side of a fluid/solid face: the no-slip wall ghost of the fluid cell ([`wall_ghost3`]).
    Wall,
    /// The exit of a motor jet: the solid cell `Part::Jet` behind the face holds the jet state (never updated), which is the ghost of the face ([`is_jet_cell`]); its
    /// viscosity, eddy viscosity and gradient are those the solver stored in the jet cell (the 2D `Side::fixed`: `mu(T_jet)`, `rho_jet k / omega`; the gradient of the fluid
    /// cell). Not a no-slip wall.
    Jet,
}

/// The two sides of a face (`l`: lower index, `r`: upper index) as indices: the fluid cell of each side (for a ghost: the adjacent fluid cell), its kind and
/// the index into the padded array `w` of the state it is made of (the cell, the domain ghost, or the fluid cell a wall ghost mirrors).
#[derive(Clone, Copy, Debug)]
pub struct FaceCells {
    pub cell: [usize; 2],
    pub kind: [SideKind; 2],
    pub pad: [usize; 2],
}

impl FaceCells {
    /// Is the face a no-slip wall (a fluid/solid face)?
    #[inline]
    pub fn noslip(&self) -> bool {
        self.kind[0] == SideKind::Wall || self.kind[1] == SideKind::Wall
    }
}

/// Is cell `c` a `Part::Jet` cell, the exit of a motor jet? It is solid (never updated) and holds the jet state, but is no wall: no reflection or no-slip ghost is built
/// at its faces. See [`SideKind::Jet`].
#[inline]
pub fn is_jet_cell(grid: &Grid3, c: usize) -> bool {
    let p = grid.part[c];
    p != PART_FLUID && p != PART_DEAD && grid.parts[usize::from(p) - 1] == Part::Jet
}

/// Sides of face `(i, j, k)` of family `dir`; `None` for a face that carries no flux: `area == 0`, between two solid cells, or on the domain boundary
/// in front of a solid cell. `solid` is the per-cell mask (`ni * nj * nk`). A `j = 0` boundary face inside a run of [`Grid3::wall_j0`] (the body surface of a
/// grid that has no solid cells under it) is a no-slip wall like a fluid/solid face: its ghost side is `Wall`.
pub fn face_cells(grid: &Grid3, solid: &[bool], dir: usize, i: usize, j: usize, k: usize) -> Option<FaceCells> {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let (_, face) = face_of(grid, dir, i, j, k);
    if face.area == 0.0 {
        return None;
    }
    let c = [i, j, k];
    let n = [ni, nj, nk];
    let idx = c[dir];
    let hi = (idx < n[dir]).then(|| grid.cell(i, j, k));
    let lo = (idx > 0).then(|| {
        let mut l = c;
        l[dir] -= 1;
        grid.cell(l[0], l[1], l[2])
    });
    let hi_p = pidx3(ni, nj, nk, i as isize, j as isize, k as isize);
    let mut l = [i as isize, j as isize, k as isize];
    l[dir] -= 1;
    let lo_p = pidx3(ni, nj, nk, l[0], l[1], l[2]);
    use SideKind::{Cell, Domain, Jet, Wall};
    match (lo, hi) {
        (Some(a), Some(b)) => match (solid[a], solid[b]) {
            (true, true) => None,
            (true, false) if is_jet_cell(grid, a) => Some(FaceCells { cell: [a, b], kind: [Jet, Cell], pad: [lo_p, hi_p] }),
            (false, true) if is_jet_cell(grid, b) => Some(FaceCells { cell: [a, b], kind: [Cell, Jet], pad: [lo_p, hi_p] }),
            (true, false) => Some(FaceCells { cell: [b, b], kind: [Wall, Cell], pad: [hi_p, hi_p] }),
            (false, true) => Some(FaceCells { cell: [a, a], kind: [Cell, Wall], pad: [lo_p, lo_p] }),
            (false, false) => Some(FaceCells { cell: [a, b], kind: [Cell, Cell], pad: [lo_p, hi_p] }),
        },
        (None, Some(b)) if solid[b] => None,
        (None, Some(b)) if dir == 1 && grid.wall_j0.iter().any(|(r, _)| r.contains(&i)) => {
            Some(FaceCells { cell: [b, b], kind: [Wall, Cell], pad: [hi_p, hi_p] })
        }
        (None, Some(b)) => Some(FaceCells { cell: [b, b], kind: [Domain, Cell], pad: [lo_p, hi_p] }),
        (Some(a), None) => (!solid[a]).then_some(FaceCells { cell: [a, a], kind: [Cell, Domain], pad: [lo_p, hi_p] }),
        (None, None) => None,
    }
}

/// One side of a face: its primitive state, the fluid cell it belongs to (for a ghost: the adjacent fluid cell) and whether it is a ghost.
#[derive(Clone, Copy, Debug)]
pub struct Side3 {
    pub w: Prim3,
    pub cell: usize,
    pub ghost: bool,
}

/// The two sides of a face with their states and whether the face is a no-slip wall.
#[derive(Clone, Copy, Debug)]
pub struct FaceSides3 {
    pub l: Side3,
    pub r: Side3,
    pub noslip: bool,
}

/// States of the two sides of face `(i, j, k)` of family `dir` (see [`face_cells`]); `w` is the padded primitive array of `Solver3`, `iso` the isothermal wall
/// condition (`None`: adiabatic; see [`wall_ghost_thermal3`]).
pub fn face_sides3(grid: &Grid3, solid: &[bool], w: &[Prim3], iso: Option<IsoWall>, dir: usize, i: usize, j: usize, k: usize) -> Option<FaceSides3> {
    let fc = face_cells(grid, solid, dir, i, j, k)?;
    let side = |s: usize| {
        let st = w[fc.pad[s]];
        Side3 { w: if fc.kind[s] == SideKind::Wall { wall_ghost_thermal3(&st, iso) } else { st }, cell: fc.cell[s], ghost: fc.kind[s] != SideKind::Cell }
    };
    Some(FaceSides3 { l: side(0), r: side(1), noslip: fc.noslip() })
}

/// Geometry of the compact gradient of one face: the unit vector `e` from the lower to the upper side, the distance `d = |d|` of the two sides (a ghost at
/// the mirror image of its cell centre in the face centroid), the no-slip flag and whether the ghost is a symmetry-plane reflection (`SymSides`).
/// `d == 0` marks a face that carries no flux (see [`face_cells`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceDir3 {
    pub e: [f64; 3],
    pub d: f64,
    pub noslip: bool,
    pub sym: bool,
}

impl FaceDir3 {
    pub const NONE: FaceDir3 = FaceDir3 { e: [0.0; 3], d: 0.0, noslip: false, sym: false };
}

/// Which of the six sides of the domain box `[i = 0, i = ni, j = 0, j = nj, k = 0, k = nk]` are symmetry planes: the ghost layer there is the reflection of
/// the cells about the face (`gas3::reflect3`), so a ghost carries the reflected gradient of its cell. Any other ghost state (far field, outflow, axis map)
/// carries the gradient of its cell.
pub type SymSides = [bool; 6];

/// [`FaceDir3`] of every face of the three families (layouts of `Grid3::{iface, jface, kface}`).
#[derive(Clone, Debug)]
pub struct FaceDirs3 {
    pub i: Vec<FaceDir3>,
    pub j: Vec<FaceDir3>,
    pub k: Vec<FaceDir3>,
}

impl FaceDirs3 {
    /// The family `dir` (0: i, 1: j, 2: k).
    #[inline]
    pub fn family(&self, dir: usize) -> &[FaceDir3] {
        match dir {
            0 => &self.i,
            1 => &self.j,
            _ => &self.k,
        }
    }
}

/// Apply `f(i, j, k, q)` to every face `(i, j, k)` (flat index `q`) of family `dir` into `out`, in parallel over the `k` planes.
pub(crate) fn map_faces<T: Send>(grid: &Grid3, dir: usize, out: &mut [T], f: impl Fn(usize, usize, usize, usize) -> T + Sync) {
    let [nfi, nfj, _] = face_shape(grid, dir);
    let plane = nfi * nfj;
    out.par_chunks_mut(plane).enumerate().for_each(|(k, po)| {
        for j in 0..nfj {
            for i in 0..nfi {
                po[j * nfi + i] = f(i, j, k, k * plane + j * nfi + i);
            }
        }
    });
}

/// Geometry of the compact face gradients of all faces: `e` and `d` from the centres of the two sides, a ghost taking the mirror image of the centre of its
/// cell in the plane of the face (through its centroid, normal `Face3::n`), so that `e` is the face normal on a wall or boundary face. Once per grid (and
/// solid mask / boundary kinds).
pub fn face_dirs(grid: &Grid3, solid: &[bool], sym: &SymSides) -> FaceDirs3 {
    let family = |dir: usize| {
        let [n0, n1, n2] = face_shape(grid, dir);
        let mut v = vec![FaceDir3::NONE; n0 * n1 * n2];
        map_faces(grid, dir, &mut v, |i, j, k, _| {
            let Some(fc) = face_cells(grid, solid, dir, i, j, k) else { return FaceDir3::NONE };
            let (_, face) = face_of(grid, dir, i, j, k);
            let pos = |s: usize| {
                // the ghost of a jet face is the jet cell's state, placed (as a wall ghost) at the mirror image of the centre of the fluid cell
                let x = grid.xc[if fc.kind[s] == SideKind::Jet { fc.cell[1 - s] } else { fc.cell[s] }];
                if fc.kind[s] == SideKind::Cell {
                    x
                } else {
                    let h = (x[0] - face.c[0]) * face.n[0] + (x[1] - face.c[1]) * face.n[1] + (x[2] - face.c[2]) * face.n[2];
                    [x[0] - 2.0 * h * face.n[0], x[1] - 2.0 * h * face.n[1], x[2] - 2.0 * h * face.n[2]]
                }
            };
            let (pl, pr) = (pos(0), pos(1));
            let d = [pr[0] - pl[0], pr[1] - pl[1], pr[2] - pl[2]];
            let dd = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            if dd == 0.0 {
                return FaceDir3::NONE;
            }
            let n = [grid.ni, grid.nj, grid.nk][dir];
            let idx = [i, j, k][dir];
            let on_sym = ((idx == 0 && sym[2 * dir]) || (idx == n && sym[2 * dir + 1])) && !fc.noslip();
            FaceDir3 { e: [d[0] / dd, d[1] / dd, d[2] / dd], d: dd, noslip: fc.noslip(), sym: on_sym }
        });
        v
    };
    FaceDirs3 { i: family(0), j: family(1), k: family(2) }
}

/// Pass 1: viscosity and Green-Gauss gradient of `(u, v, w, T)` of every fluid cell (zero in solid cells). The neighbour across a no-slip wall is the wall
/// ghost, so the face value of the velocity there is 0; a face without flux (zero area) adds nothing.
#[allow(clippy::too_many_arguments)]
pub fn cell_pass(v: &Viscous, grid: &Grid3, solid: &[bool], w: &[Prim3], iso: Option<IsoWall>, grad: &mut [Grad3], mu: &mut [f64]) {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let plane = ni * nj;
    grad.par_chunks_mut(plane).zip(mu.par_chunks_mut(plane)).enumerate().for_each(|(k, (gpl, mpl))| {
        for j in 0..nj {
            for i in 0..ni {
                let c = grid.cell(i, j, k);
                if solid[c] {
                    gpl[j * ni + i] = [0.0; 12];
                    mpl[j * ni + i] = 0.0;
                    continue;
                }
                let wc = &w[pidx3(ni, nj, nk, i as isize, j as isize, k as isize)];
                mpl[j * ni + i] = v.mu(temperature3(wc));
                let pc = phi3(wc);
                let mut g = [0.0; 12];
                for dir in 0..3 {
                    for hi in [false, true] {
                        let mut f = [i, j, k];
                        f[dir] += hi as usize;
                        let (_, face) = face_of(grid, dir, f[0], f[1], f[2]);
                        let Some(s) = face_sides3(grid, solid, w, iso, dir, f[0], f[1], f[2]) else { continue };
                        let pn = phi3(&if hi { s.r.w } else { s.l.w });
                        let sign = if hi { 1.0 } else { -1.0 };
                        for m in 0..4 {
                            let pf = 0.5 * (pc[m] + pn[m]);
                            for a in 0..3 {
                                g[3 * m + a] += sign * pf * face.n[a] * face.area;
                            }
                        }
                    }
                }
                let inv_v = 1.0 / grid.vol[c];
                for x in g.iter_mut() {
                    *x *= inv_v;
                }
                gpl[j * ni + i] = g;
            }
        }
    });
}

/// Face viscosity and eddy viscosity: mean of the two sides, a ghost taking its cell's values (eddy viscosity `2 mu_t,w - mu_t` across a no-slip face, `-mu_t`
/// for `wall` empty: the face value is the wall eddy viscosity `mu_t,w = wall[cell][2]`, 0 when smooth). Across an isothermal wall (`iso`) the laminar ghost viscosity
/// is `2 mu(T_w) - mu`, so the face value is `mu(T_w)`, the viscosity at the wall.
#[inline]
pub fn face_mu3(mu: &[f64], mu_t: &[f64], wall: &[[f64; 3]], iso: Option<IsoWall>, sides: &FaceSides3) -> (f64, f64) {
    face_mu_cells(mu, mu_t, wall, iso, [sides.l.cell, sides.r.cell], [sides.l.ghost, sides.r.ghost], sides.noslip)
}

/// [`face_mu3`] from the fluid cells of the two sides (`cell`, for a ghost the adjacent fluid cell) and their ghost flags.
#[inline]
pub fn face_mu_cells(mu: &[f64], mu_t: &[f64], wall: &[[f64; 3]], iso: Option<IsoWall>, cell: [usize; 2], ghost: [bool; 2], noslip: bool) -> (f64, f64) {
    let mut_side = |s: usize| {
        let c = cell[s];
        if ghost[s] && noslip {
            if wall.is_empty() {
                -mu_t[c]
            } else {
                2.0 * wall[c][2] - mu_t[c]
            }
        } else {
            mu_t[c]
        }
    };
    let mu_side = |s: usize| match iso {
        Some(i) if ghost[s] && noslip => 2.0 * i.mu_w - mu[cell[s]],
        _ => mu[cell[s]],
    };
    (0.5 * (mu_side(0) + mu_side(1)), 0.5 * (mut_side(0) + mut_side(1)))
}

/// The gradient record of the mirror image of the field about the plane with unit normal `n`: the field `phi_g(x) = R phi(R x)` of a symmetry-plane ghost
/// (`R = I - 2 n n^T`, the velocity vector reflected, the scalars copied) has the gradient `R G R` (velocity) and `R grad T` (temperature).
#[inline]
pub fn reflect_grad(g: &Grad3, n: [f64; 3]) -> Grad3 {
    let r = |a: usize, b: usize| (a == b) as usize as f64 - 2.0 * n[a] * n[b];
    let mut out = [0.0; 12];
    for a in 0..3 {
        for b in 0..3 {
            for c in 0..3 {
                for d in 0..3 {
                    out[3 * a + b] += r(a, c) * g[3 * c + d] * r(d, b);
                }
            }
        }
        for c in 0..3 {
            out[9 + a] += r(a, c) * g[9 + c];
        }
    }
    out
}

/// Face gradient of `(u, v, w, T)`: `g_f = gbar + ((phi_R - phi_L) / d - gbar.e) e` per quantity, `gbar` the mean of the two side gradients. A ghost carries the
/// gradient of its cell, except across a symmetry plane (`dir.sym`, face normal `n`) where it carries the reflected one ([`reflect_grad`]): the face tensor is
/// then the average of the tensors of the cell and of its mirror image, free of the shear `n.G.t` that the unreflected average would leave on a plane that
/// cuts a curved flow (on a wedge of opening angle `dtheta` the error `O(dtheta)` of the face stress is divided by the cell thickness `r dtheta`).
#[inline]
pub fn face_gradient(dir: &FaceDir3, n: [f64; 3], grad: &[Grad3], sides: &FaceSides3) -> Grad3 {
    let side_grad = |s: &Side3| if dir.sym && s.ghost { reflect_grad(&grad[s.cell], n) } else { grad[s.cell] };
    let (gl, gr) = (side_grad(&sides.l), side_grad(&sides.r));
    let (pl, pr) = (phi3(&sides.l.w), phi3(&sides.r.w));
    let e = dir.e;
    let mut gf = [0.0; 12];
    for m in 0..4 {
        let gb = [0.5 * (gl[3 * m] + gr[3 * m]), 0.5 * (gl[3 * m + 1] + gr[3 * m + 1]), 0.5 * (gl[3 * m + 2] + gr[3 * m + 2])];
        let corr = (pr[m] - pl[m]) / dir.d - (gb[0] * e[0] + gb[1] * e[1] + gb[2] * e[2]);
        for a in 0..3 {
            gf[3 * m + a] = gb[a] + corr * e[a];
        }
    }
    gf
}

/// Viscous flux `Fv.n` of one face (per unit area; see the module documentation); `[0; 5]` for a face without flux (`dir.d == 0`).
#[allow(clippy::too_many_arguments)]
pub fn viscous_flux3(
    face: &Face3,
    dir: &FaceDir3,
    grad: &[Grad3],
    mu: &[f64],
    mu_t: &[f64],
    wall: &[[f64; 3]],
    iso: Option<IsoWall>,
    sides: &FaceSides3,
) -> Cons3 {
    let gf = face_gradient(dir, face.n, grad, sides);
    let (mu_f, mut_f) = face_mu3(mu, mu_t, wall, iso, sides);
    let uf = [0.5 * (sides.l.w[1] + sides.r.w[1]), 0.5 * (sides.l.w[2] + sides.r.w[2]), 0.5 * (sides.l.w[3] + sides.r.w[3])];
    let div = gf[0] + gf[4] + gf[8];
    let mue = mu_f + mut_f;
    let kappa = (mu_f / PRANDTL + mut_f / PRANDTL_T) / (GAMMA - 1.0);
    let n = face.n;
    // t_i = tau_ij n_j
    let mut t = [0.0; 3];
    for (a, ta) in t.iter_mut().enumerate() {
        for (b, nb) in n.iter().enumerate() {
            let tau = mue * (gf[3 * a + b] + gf[3 * b + a] - if a == b { 2.0 / 3.0 * div } else { 0.0 });
            *ta += tau * nb;
        }
    }
    let heat = kappa * (gf[9] * n[0] + gf[10] * n[1] + gf[11] * n[2]);
    [0.0, t[0], t[1], t[2], uf[0] * t[0] + uf[1] * t[1] + uf[2] * t[2] + heat]
}

/// Pass 2: viscous fluxes `Fv.n` (the mass component is 0) of every i-face (`fvi`), j-face (`fvj`) and k-face (`fvk`), layouts of `Grid3::{iface, jface, kface}`;
/// zero for a face without flux. `wall` is the per-cell wall state `[k_w, omega_w, mu_t,w]` of the turbulence wall treatment (empty: smooth walls, `mu_t,w = 0`),
/// `mu_t` the eddy viscosity per cell (zeros for a laminar run).
#[allow(clippy::too_many_arguments)]
pub fn face_pass(
    grid: &Grid3,
    solid: &[bool],
    w: &[Prim3],
    dirs: &FaceDirs3,
    grad: &[Grad3],
    mu: &[f64],
    mu_t: &[f64],
    wall: &[[f64; 3]],
    iso: Option<IsoWall>,
    fvi: &mut [Cons3],
    fvj: &mut [Cons3],
    fvk: &mut [Cons3],
) {
    for (dir, out) in [(0, fvi), (1, fvj), (2, fvk)] {
        let fam = dirs.family(dir);
        map_faces(grid, dir, out, |i, j, k, q| {
            if fam[q].d == 0.0 {
                return [0.0; 5];
            }
            let s = face_sides3(grid, solid, w, iso, dir, i, j, k).expect("a face with a direction carries flux");
            viscous_flux3(face_of(grid, dir, i, j, k).1, &fam[q], grad, mu, mu_t, wall, iso, &s)
        });
    }
}

/// Adds the viscous part of the residual `dU/dt += (1 / V) sum_faces +-(Fv.n) A` to the fluid cells of `res` (solid cells untouched).
pub fn add_viscous_residual(grid: &Grid3, solid: &[bool], fvi: &[Cons3], fvj: &[Cons3], fvk: &[Cons3], res: &mut [Cons3]) {
    let (ni, nj) = (grid.ni, grid.nj);
    res.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let c = grid.cell(i, j, k);
                if solid[c] {
                    continue;
                }
                let (il, jl) = ((k * nj + j) * (ni + 1) + i, (k * (nj + 1) + j) * ni + i);
                let (ir, jr, kr) = (il + 1, jl + ni, c + ni * nj);
                let inv_v = 1.0 / grid.vol[c];
                let r = &mut pl[j * ni + i];
                for q in 0..5 {
                    r[q] += ((fvi[ir][q] * grid.iface[ir].area - fvi[il][q] * grid.iface[il].area)
                        + (fvj[jr][q] * grid.jface[jr].area - fvj[jl][q] * grid.jface[jl].area)
                        + (fvk[kr][q] * grid.kface[kr].area - fvk[c][q] * grid.kface[c].area))
                        * inv_v;
                }
            }
        }
    });
}

/// The viscous force on the body at one no-slip face: the family `dir` and index `idx` of the face (as [`face_of`]), its `area`, the unit normal `n` out of the
/// fluid into the solid, and the force per unit area `tau` that the fluid exerts on the wall by viscous stress (shear and the small normal viscous stress).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallTraction {
    pub dir: usize,
    pub idx: [usize; 3],
    pub area: f64,
    pub n: [f64; 3],
    pub tau: [f64; 3],
}

/// The viscous traction of every no-slip face from the face fluxes `fvi, fvj, fvk` of [`face_pass`]: the residual adds `+(Fv.n) A` to the momentum of a fluid
/// cell through its upper face and `-(Fv.n) A` through its lower one, so the force on the wall is `tau = -sigma Fv.n[1..4]` with `sigma = +1` where the fluid cell
/// is the lower side of the face and `-1` where it is the upper one (`n = sigma n_face`). The momentum flux the mean-flow equations see (face viscosity
/// `mu + mu_t,w` of the wall treatment) is therefore the very force the body feels: the friction drag is `sum tau.e_x area`, no second shear formula.
pub fn wall_tractions(grid: &Grid3, solid: &[bool], dirs: &FaceDirs3, fvi: &[Cons3], fvj: &[Cons3], fvk: &[Cons3]) -> Vec<WallTraction> {
    let mut out = Vec::new();
    for (dir, fv) in [(0, fvi), (1, fvj), (2, fvk)] {
        let [n0, n1, n2] = face_shape(grid, dir);
        for k in 0..n2 {
            for j in 0..n1 {
                for i in 0..n0 {
                    let (q, face) = face_of(grid, dir, i, j, k);
                    if !dirs.family(dir)[q].noslip {
                        continue;
                    }
                    let Some(fc) = face_cells(grid, solid, dir, i, j, k) else { continue };
                    let sigma = if fc.kind[0] == SideKind::Cell { 1.0 } else { -1.0 };
                    let f = &fv[q];
                    out.push(WallTraction {
                        dir,
                        idx: [i, j, k],
                        area: face.area,
                        n: [sigma * face.n[0], sigma * face.n[1], sigma * face.n[2]],
                        tau: [-sigma * f[1], -sigma * f[2], -sigma * f[3]],
                    });
                }
            }
        }
    }
    out
}

/// Diffusive contribution `2 lv A_f^2 / V` to the spectral radius `L_d` of one direction (`area`: mean area of the two faces of that direction, `vol` the cell
/// volume), with `lv = gamma (mu / PRANDTL + mu_t / PRANDTL_T) / rho`. The 2D [`crate::viscous::diffusion_rate`] with face length -> area, cell area -> volume.
#[inline]
pub fn diffusion_rate(mu_c: f64, mu_t_c: f64, rho_c: f64, area: f64, vol: f64) -> f64 {
    let lv = GAMMA * (mu_c / PRANDTL + mu_t_c / PRANDTL_T) / rho_c;
    2.0 * lv * area * area / vol
}

/// [`diffusion_rate`] of the three directions `[i, j, k]` of cell `(i, j, k)` (mean area of its two faces per direction).
pub fn diffusion_rates(grid: &Grid3, i: usize, j: usize, k: usize, mu_c: f64, mu_t_c: f64, rho_c: f64) -> [f64; 3] {
    let vol = grid.vol[grid.cell(i, j, k)];
    [0, 1, 2].map(|dir| {
        let mut up = [i, j, k];
        up[dir] += 1;
        let (a, b) = (face_of(grid, dir, i, j, k).1.area, face_of(grid, dir, up[0], up[1], up[2]).1.area);
        diffusion_rate(mu_c, mu_t_c, rho_c, 0.5 * (a + b), vol)
    })
}

/// Diffusive spectral radius `lam = gamma (mu_f / PRANDTL + mu_t,f / PRANDTL_T) / (rho_f |d|)` of face `(i, j, k)` of family `dir` (`0.0` for a face without
/// flux): the implicit operator adds `2 lam I` to the dissipation matrix of the face (`rho_f` the mean density of its two sides).
#[allow(clippy::too_many_arguments)]
pub fn face_lambda3(
    grid: &Grid3,
    solid: &[bool],
    w: &[Prim3],
    dirs: &FaceDirs3,
    mu: &[f64],
    mu_t: &[f64],
    wall: &[[f64; 3]],
    iso: Option<IsoWall>,
    dir: usize,
    i: usize,
    j: usize,
    k: usize,
) -> f64 {
    let (q, _) = face_of(grid, dir, i, j, k);
    let d = dirs.family(dir)[q].d;
    if d == 0.0 {
        return 0.0;
    }
    let s = face_sides3(grid, solid, w, iso, dir, i, j, k).expect("a face with a direction carries flux");
    let (mu_f, mut_f) = face_mu3(mu, mu_t, wall, iso, &s);
    let rho_f = 0.5 * (s.l.w[0] + s.r.w[0]);
    GAMMA * (mu_f / PRANDTL + mut_f / PRANDTL_T) / (rho_f * d)
}

/// [`face_lambda3`] of the lower and upper face of cell `(i, j, k)` in the direction `dir` (the 2D `viscous::face_lambdas`).
#[allow(clippy::too_many_arguments)]
pub fn face_lambdas(
    grid: &Grid3,
    solid: &[bool],
    w: &[Prim3],
    dirs: &FaceDirs3,
    mu: &[f64],
    mu_t: &[f64],
    wall: &[[f64; 3]],
    iso: Option<IsoWall>,
    dir: usize,
    i: usize,
    j: usize,
    k: usize,
) -> (f64, f64) {
    let mut up = [i, j, k];
    up[dir] += 1;
    (face_lambda3(grid, solid, w, dirs, mu, mu_t, wall, iso, dir, i, j, k), face_lambda3(grid, solid, w, dirs, mu, mu_t, wall, iso, dir, up[0], up[1], up[2]))
}

/// [`face_lambda3`] of every face of the three families (layouts of `Grid3::{iface, jface, kface}`).
#[allow(clippy::too_many_arguments)]
pub fn lambda_pass(
    grid: &Grid3,
    solid: &[bool],
    w: &[Prim3],
    dirs: &FaceDirs3,
    mu: &[f64],
    mu_t: &[f64],
    wall: &[[f64; 3]],
    iso: Option<IsoWall>,
    li: &mut [f64],
    lj: &mut [f64],
    lk: &mut [f64],
) {
    for (dir, out) in [(0, li), (1, lj), (2, lk)] {
        map_faces(grid, dir, out, |i, j, k, _| face_lambda3(grid, solid, w, dirs, mu, mu_t, wall, iso, dir, i, j, k));
    }
}

#[cfg(test)]
pub(crate) mod wedge;

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::flux3::tests::{random_rotation, rot, Rng};
    use crate::grid3::{Part, SolidBox};
    use crate::solver3::NG;

    const RE_C: f64 = 2.0e3;
    pub(crate) const VISC: Viscous = Viscous { re_c: RE_C, sutherland: 0.368, turbulence: crate::sst::Turbulence::Laminar, wall_i_start: 0, roughness: 0.0 };

    /// Per-cell solid mask of a grid (`Grid3::is_solid` of every cell, the `Solver3::solid` array).
    pub(crate) fn solid_mask(g: &Grid3) -> Vec<bool> {
        (0..g.ni * g.nj * g.nk).map(|c| g.is_solid(c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj))).collect()
    }

    /// Declare the boxes solid walls of the grid (`Grid3::mark_solid`, which fills the per-cell `part` that `Grid3::is_solid` and `Solver3` read).
    pub(crate) fn set_solids(g: &mut Grid3, boxes: Vec<SolidBox>) {
        for b in boxes {
            g.mark_solid(b, Some(Part::Body(0)));
        }
    }

    /// Padded primitive array: interior cells from `cells`, every ghost cell a copy of the nearest interior cell, solid cells NaN (a kernel that
    /// reads one shows up as NaN in its output).
    pub(crate) fn padded(g: &Grid3, solid: &[bool], cells: &[Prim3]) -> Vec<Prim3> {
        let (ni, nj, nk) = (g.ni, g.nj, g.nk);
        let mut w = vec![[f64::NAN; 5]; (ni + 2 * NG) * (nj + 2 * NG) * (nk + 2 * NG)];
        let ng = NG as isize;
        for k in -ng..nk as isize + ng {
            for j in -ng..nj as isize + ng {
                for i in -ng..ni as isize + ng {
                    let (ci, cj, ck) = (i.clamp(0, ni as isize - 1) as usize, j.clamp(0, nj as isize - 1) as usize, k.clamp(0, nk as isize - 1) as usize);
                    let c = g.cell(ci, cj, ck);
                    if !solid[c] {
                        w[pidx3(ni, nj, nk, i, j, k)] = cells[c];
                    }
                }
            }
        }
        w
    }

    /// Run the three passes on a state with no symmetry-plane ghosts; returns `(grad, mu, fvi, fvj, fvk)`.
    #[allow(clippy::type_complexity)]
    pub(crate) fn run_passes(
        g: &Grid3,
        solid: &[bool],
        w: &[Prim3],
        mu_t: &[f64],
        wall: &[[f64; 3]],
    ) -> (Vec<Grad3>, Vec<f64>, Vec<Cons3>, Vec<Cons3>, Vec<Cons3>) {
        run_passes_sym(g, solid, w, mu_t, wall, &[false; 6])
    }

    /// [`run_passes`] with the symmetry sides of the domain given.
    #[allow(clippy::type_complexity)]
    pub(crate) fn run_passes_sym(
        g: &Grid3,
        solid: &[bool],
        w: &[Prim3],
        mu_t: &[f64],
        wall: &[[f64; 3]],
        sym: &SymSides,
    ) -> (Vec<Grad3>, Vec<f64>, Vec<Cons3>, Vec<Cons3>, Vec<Cons3>) {
        let nc = g.ni * g.nj * g.nk;
        let (mut grad, mut mu) = (vec![[0.0; 12]; nc], vec![0.0; nc]);
        cell_pass(&VISC, g, solid, w, None, &mut grad, &mut mu);
        let dirs = face_dirs(g, solid, sym);
        let (mut fvi, mut fvj, mut fvk) = (vec![[0.0; 5]; g.iface.len()], vec![[0.0; 5]; g.jface.len()], vec![[0.0; 5]; g.kface.len()]);
        face_pass(g, solid, w, &dirs, &grad, &mu, mu_t, wall, None, &mut fvi, &mut fvj, &mut fvk);
        (grad, mu, fvi, fvj, fvk)
    }

    pub(crate) fn box_grid(n: [usize; 3], h: [f64; 3]) -> Grid3 {
        let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    x.push(i as f64 * h[0]);
                    y.push(j as f64 * h[1]);
                    z.push(k as f64 * h[2]);
                }
            }
        }
        Grid3::try_from_nodes(n[0], n[1], n[2], x, y, z).unwrap()
    }

    fn close(a: f64, b: f64, tol: f64, what: &str) {
        assert!((a - b).abs() <= tol * (1.0 + b.abs()), "{what}: {a:e} vs {b:e}");
    }

    /// Linear fields on a uniform (non-cubic) Cartesian box: the cell gradients and the face fluxes are exact on every cell and face, boundary ones
    /// included (the ghosts hold the continued linear field), the flux with the face viscosity `mean(mu_L, mu_R)` of the two sides.
    #[test]
    fn linear_fields_on_a_uniform_box_are_exact() {
        let h = [0.11, 0.07, 0.09];
        let n = [6, 5, 4];
        let g = box_grid(n, h);
        let solid = vec![false; g.ni * g.nj * g.nk];
        let mut rng = Rng::new(21);
        let gv = [
            [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)],
            [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)],
            [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)],
        ];
        let gt = [rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5)];
        let u0 = [0.3, -0.2, 0.1];
        let state = |x: [f64; 3]| -> Prim3 {
            let vel: Vec<f64> = (0..3).map(|a| u0[a] + (0..3).map(|b| gv[a][b] * x[b]).sum::<f64>()).collect();
            let t = 1.0 + (0..3).map(|b| gt[b] * x[b]).sum::<f64>();
            let rho = 1.2;
            [rho, vel[0], vel[1], vel[2], rho * t / GAMMA]
        };
        let centre = |i: isize, j: isize, k: isize| [(i as f64 + 0.5) * h[0], (j as f64 + 0.5) * h[1], (k as f64 + 0.5) * h[2]];
        // padded array from the analytic field at every cell centre, ghosts included
        let (ni, nj, nk) = (n[0], n[1], n[2]);
        let mut w = vec![[0.0; 5]; (ni + 2 * NG) * (nj + 2 * NG) * (nk + 2 * NG)];
        let ng = NG as isize;
        for k in -ng..nk as isize + ng {
            for j in -ng..nj as isize + ng {
                for i in -ng..ni as isize + ng {
                    w[pidx3(ni, nj, nk, i, j, k)] = state(centre(i, j, k));
                }
            }
        }
        let mu_t = vec![0.0; solid.len()];
        let (grad, mu, fvi, fvj, fvk) = run_passes(&g, &solid, &w, &mu_t, &[]);
        let mut want_g = [0.0; 12];
        for a in 0..3 {
            for b in 0..3 {
                want_g[3 * a + b] = gv[a][b];
            }
            want_g[9 + a] = gt[a];
        }
        for (c, gc) in grad.iter().enumerate() {
            for q in 0..12 {
                close(gc[q], want_g[q], 1e-12, &format!("grad cell {c} comp {q}"));
            }
        }
        let div = gv[0][0] + gv[1][1] + gv[2][2];
        for (dir, fam) in [(0, &fvi), (1, &fvj), (2, &fvk)] {
            let [n0, n1, n2] = face_shape(&g, dir);
            for k in 0..n2 {
                for j in 0..n1 {
                    for i in 0..n0 {
                        let (q, face) = face_of(&g, dir, i, j, k);
                        // the two sides of the face
                        let mut lo = [i as isize, j as isize, k as isize];
                        let hi = lo;
                        lo[dir] -= 1;
                        let (xl, xr) = (centre(lo[0], lo[1], lo[2]), centre(hi[0], hi[1], hi[2]));
                        let (pl, pr) = (state(xl), state(xr));
                        // a ghost carries the viscosity of its fluid cell
                        let idx = [i, j, k][dir];
                        let m_of = |p: &Prim3| VISC.mu(temperature3(p));
                        let (ml, mr) = if idx == 0 {
                            (m_of(&pr), m_of(&pr))
                        } else if idx == n[dir] {
                            (m_of(&pl), m_of(&pl))
                        } else {
                            (m_of(&pl), m_of(&pr))
                        };
                        let mu_f = 0.5 * (ml + mr);
                        let uf = state(face.c);
                        let nn = face.n;
                        let mut t = [0.0; 3];
                        for a in 0..3 {
                            for b in 0..3 {
                                t[a] += mu_f * (gv[a][b] + gv[b][a] - if a == b { 2.0 / 3.0 * div } else { 0.0 }) * nn[b];
                            }
                        }
                        let heat = mu_f / PRANDTL / (GAMMA - 1.0) * (gt[0] * nn[0] + gt[1] * nn[1] + gt[2] * nn[2]);
                        let want = [0.0, t[0], t[1], t[2], uf[1] * t[0] + uf[2] * t[1] + uf[3] * t[2] + heat];
                        for m in 0..5 {
                            close(fam[q][m], want[m], 1e-12, &format!("flux dir {dir} face ({i},{j},{k}) comp {m}"));
                        }
                    }
                }
            }
        }
        assert!(mu.iter().all(|m| *m > 0.0));
    }

    /// Constant viscosity (uniform `T`) and a linear velocity field: constant stress, so the viscous residual vanishes in every cell.
    #[test]
    fn constant_stress_has_no_divergence() {
        let h = [0.1, 0.12, 0.08];
        let g = box_grid([5, 4, 6], h);
        let solid = vec![false; g.ni * g.nj * g.nk];
        let state = |x: [f64; 3]| -> Prim3 { [1.0, 0.2 + 0.5 * x[1] - 0.3 * x[2], -0.1 + 0.4 * x[0], 0.3 * x[1] + 0.2 * x[2], 1.0 / GAMMA] };
        let (ni, nj, nk) = (g.ni, g.nj, g.nk);
        let mut w = vec![[0.0; 5]; (ni + 2 * NG) * (nj + 2 * NG) * (nk + 2 * NG)];
        let ng = NG as isize;
        for k in -ng..nk as isize + ng {
            for j in -ng..nj as isize + ng {
                for i in -ng..ni as isize + ng {
                    w[pidx3(ni, nj, nk, i, j, k)] = state([(i as f64 + 0.5) * h[0], (j as f64 + 0.5) * h[1], (k as f64 + 0.5) * h[2]]);
                }
            }
        }
        let mu_t = vec![0.0; solid.len()];
        let (_, _, fvi, fvj, fvk) = run_passes(&g, &solid, &w, &mu_t, &[]);
        let mut res = vec![[0.0; 5]; solid.len()];
        add_viscous_residual(&g, &solid, &fvi, &fvj, &fvk, &mut res);
        // momentum fluxes are constant; the energy flux is not (u . tau varies), only momentum is checked
        for r in &res {
            for m in 1..4 {
                assert!(r[m].abs() < 1e-12, "{r:?}");
            }
        }
    }

    /// Couette flow above a stationary no-slip wall (a solid row of cells): `u = U y / H` is reproduced exactly, the wall shear is `mu U / H`, the face
    /// velocity on the wall is 0 (no heat from `u . tau` there) and the residual of the momentum vanishes.
    #[test]
    fn couette_flow_has_exact_wall_shear() {
        let (ni, nj, nk) = (4, 8, 3);
        let h = [0.2, 0.05, 0.2];
        let mut g = box_grid([ni, nj, nk], h);
        set_solids(&mut g, vec![SolidBox { i: 0..ni, j: 0..1, k: 0..nk }]);
        let solid = solid_mask(&g);
        let (u_top, hh) = (0.6, (nj - 1) as f64 * h[1]);
        let state = |y: f64| -> Prim3 { [1.0, u_top * (y - h[1]) / hh, 0.0, 0.0, 1.0 / GAMMA] };
        let cells: Vec<Prim3> = (0..ni * nj * nk).map(|c| state(g.xc[c][1])).collect();
        let mut w = padded(&g, &solid, &cells);
        // top boundary: the continued linear profile
        for k in 0..nk as isize {
            for i in 0..ni as isize {
                for m in 0..NG as isize {
                    w[pidx3(ni, nj, nk, i, nj as isize + m, k)] = state((nj as f64 + m as f64 + 0.5) * h[1]);
                }
            }
        }
        let mu_t = vec![0.0; solid.len()];
        let (grad, mu, fvi, fvj, fvk) = run_passes(&g, &solid, &w, &mu_t, &[]);
        let tau_w = mu[g.cell(0, 1, 0)] * u_top / hh;
        let mut res = vec![[0.0; 5]; solid.len()];
        add_viscous_residual(&g, &solid, &fvi, &fvj, &fvk, &mut res);
        for k in 0..nk {
            for i in 0..ni {
                let (q, face) = face_of(&g, 1, i, 1, k);
                assert_eq!(face.n, [0.0, 1.0, 0.0]);
                close(fvj[q][1], tau_w, 1e-12, "wall shear");
                assert!(fvj[q][2].abs() < 1e-14 && fvj[q][3].abs() < 1e-14);
                assert!(fvj[q][4].abs() < 1e-14, "u_f = 0 on the wall: no work, no heat: {}", fvj[q][4]);
                // the solid row carries no flux on its lower side and no gradient
                let (q0, _) = face_of(&g, 1, i, 0, k);
                assert_eq!(fvj[q0], [0.0; 5]);
                assert_eq!(grad[g.cell(i, 0, k)], [0.0; 12]);
                close(grad[g.cell(i, 1, k)][1], u_top / hh, 1e-12, "u_y of the first fluid cell");
                for j in 1..nj {
                    let r = res[g.cell(i, j, k)];
                    assert!(r[1].abs() < 1e-11 && r[2].abs() < 1e-13 && r[3].abs() < 1e-13, "{r:?}");
                }
                assert_eq!(res[g.cell(i, 0, k)], [0.0; 5]);
            }
        }
    }

    /// An isothermal wall under a uniform fluid at rest at `T = 1.5` with `T_w = 1`: the ghost temperature is `2 T_w - T = 0.5` at the cell's pressure (floored at
    /// `ISO_GHOST_T_MIN T_w` for a hot cell), the face value of `T` is `T_w`, the wall flux is `kappa(T_w) 2 (T - T_w) / h` (the compact gradient across `h / 2`, the viscosity `mu(T_w)`
    /// of the face) and nothing else moves; the adiabatic wall of the same state has no flux at all.
    #[test]
    fn isothermal_wall_conducts_with_the_wall_viscosity() {
        let (ni, nj, nk) = (3, 5, 2);
        let h = [0.2, 0.05, 0.2];
        let mut g = box_grid([ni, nj, nk], h);
        set_solids(&mut g, vec![SolidBox { i: 0..ni, j: 0..1, k: 0..nk }]);
        let solid = solid_mask(&g);
        let (rho, t_cell, t_wall) = (1.0, 1.5, 1.0);
        let cells = vec![[rho, 0.0, 0.0, 0.0, rho * t_cell / GAMMA]; ni * nj * nk];
        let w = padded(&g, &solid, &cells);
        let iso = crate::sst::WallThermal::Isothermal(t_wall).iso(&VISC).expect("isothermal");
        assert!((iso.mu_w - VISC.mu(t_wall)).abs() < 1e-15);
        let ghost = wall_ghost_thermal3(&cells[0], Some(iso));
        assert!((temperature3(&ghost) - 0.5).abs() < 1e-14 && ghost[4] == cells[0][4] && ghost[1] == 0.0, "{ghost:?}");
        let hot = [rho, 0.0, 0.0, 0.0, rho * 3.0 / GAMMA];
        assert!((temperature3(&wall_ghost_thermal3(&hot, Some(iso))) - crate::sst::ISO_GHOST_T_MIN * t_wall).abs() < 1e-14, "the ghost temperature is floored");
        assert_eq!(wall_ghost_thermal3(&hot, None), wall_ghost3(&hot));

        let run = |iso: Option<IsoWall>| {
            let nc = ni * nj * nk;
            let (mut grad, mut mu) = (vec![[0.0; 12] as Grad3; nc], vec![0.0; nc]);
            cell_pass(&VISC, &g, &solid, &w, iso, &mut grad, &mut mu);
            let dirs = face_dirs(&g, &solid, &[false; 6]);
            let (mut fvi, mut fvj, mut fvk) = (vec![[0.0; 5]; g.iface.len()], vec![[0.0; 5]; g.jface.len()], vec![[0.0; 5]; g.kface.len()]);
            face_pass(&g, &solid, &w, &dirs, &grad, &mu, &vec![0.0; nc], &[], iso, &mut fvi, &mut fvj, &mut fvk);
            (grad, fvj)
        };
        let (grad_c, fvj_c) = run(Some(iso));
        let (grad_a, fvj_a) = run(None);
        let want = iso.mu_w / PRANDTL / (GAMMA - 1.0) * 2.0 * (t_cell - t_wall) / h[1];
        for k in 0..nk {
            for i in 0..ni {
                let (q, face) = face_of(&g, 1, i, 1, k);
                assert_eq!(face.n, [0.0, 1.0, 0.0]);
                close(fvj_c[q][4], want, 1e-12, "cold wall heat flux");
                assert!(fvj_c[q][1].abs() < 1e-14 && fvj_c[q][2].abs() < 1e-14 && fvj_c[q][3].abs() < 1e-14);
                close(grad_c[g.cell(i, 1, k)][10], (t_cell - t_wall) / h[1], 1e-12, "T_y of the first fluid cell: the face value is T_w");
                assert_eq!(fvj_a[q][4], 0.0, "adiabatic: no heat flux");
                assert_eq!(grad_a[g.cell(i, 1, k)][10], 0.0);
            }
        }
    }

    /// Jittered grid with a solid block, a random non-smooth state, eddy viscosity and wall states.
    #[allow(clippy::type_complexity)]
    fn random_case(rng: &mut Rng) -> (Vec<[f64; 3]>, [usize; 3], Vec<Prim3>, Vec<f64>, Vec<[f64; 3]>) {
        let n = [6, 5, 4];
        let mut nodes = vec![];
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    let jit = |r: &mut Rng| r.range(-0.12, 0.12);
                    nodes.push([0.1 * i as f64 + 0.1 * jit(rng), 0.08 * j as f64 + 0.08 * jit(rng) + 0.02 * i as f64, 0.09 * k as f64 + 0.09 * jit(rng)]);
                }
            }
        }
        let nc = n[0] * n[1] * n[2];
        let cells: Vec<Prim3> =
            (0..nc).map(|_| [rng.range(0.8, 1.2), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(0.6, 1.2)]).collect();
        let mu_t: Vec<f64> = (0..nc).map(|_| rng.range(0.0, 3.0e-3)).collect();
        let wall: Vec<[f64; 3]> = (0..nc).map(|_| [rng.range(0.0, 1.0), rng.range(0.0, 1.0), rng.range(0.0, 2.0e-3)]).collect();
        (nodes, n, cells, mu_t, wall)
    }

    fn grid_of(nodes: &[[f64; 3]], n: [usize; 3]) -> Grid3 {
        let mut g = Grid3::try_from_nodes(
            n[0],
            n[1],
            n[2],
            nodes.iter().map(|p| p[0]).collect(),
            nodes.iter().map(|p| p[1]).collect(),
            nodes.iter().map(|p| p[2]).collect(),
        )
        .unwrap();
        set_solids(&mut g, vec![SolidBox { i: 2..4, j: 1..3, k: 1..2 }]);
        g
    }

    fn assert_no_nan(r: &[Cons3], what: &str) {
        assert!(r.iter().all(|f| f.iter().all(|x| x.is_finite())), "{what} has a non-finite entry");
    }

    /// Rotating or mirroring the grid and the velocity vectors rotates or mirrors the viscous fluxes (momentum components), and leaves the energy flux and
    /// the cell viscosity unchanged. Tensor contractions, the wall ghost across a solid block and the ghost viscosities are all exercised; `w` of the solid
    /// cells is NaN.
    #[test]
    fn rotated_and_mirrored_states_give_rotated_and_mirrored_fluxes() {
        let mut rng = Rng::new(7);
        for trial in 0..8 {
            let (nodes, n, cells, mu_t, wall) = random_case(&mut rng);
            let g = grid_of(&nodes, n);
            let solid = solid_mask(&g);
            let w = padded(&g, &solid, &cells);
            let (grad, mu, fvi, fvj, fvk) = run_passes(&g, &solid, &w, &mu_t, &wall);
            assert_no_nan(&fvi, "fvi");
            assert_no_nan(&fvj, "fvj");
            assert_no_nan(&fvk, "fvk");
            // rotation (trial 0..4) or rotation followed by the reflection about a random plane (4..8)
            let mut q = random_rotation(&mut rng);
            if trial >= 4 {
                let m = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
                let l = (m[0] * m[0] + m[1] * m[1] + m[2] * m[2]).sqrt();
                let m = [m[0] / l, m[1] / l, m[2] / l];
                let refl: crate::flux3::tests::Rot = [
                    [1.0 - 2.0 * m[0] * m[0], -2.0 * m[0] * m[1], -2.0 * m[0] * m[2]],
                    [-2.0 * m[1] * m[0], 1.0 - 2.0 * m[1] * m[1], -2.0 * m[1] * m[2]],
                    [-2.0 * m[2] * m[0], -2.0 * m[2] * m[1], 1.0 - 2.0 * m[2] * m[2]],
                ];
                let mut p = [[0.0; 3]; 3];
                for a in 0..3 {
                    for b in 0..3 {
                        p[a][b] = (0..3).map(|c| refl[a][c] * q[c][b]).sum();
                    }
                }
                q = p;
            }
            let nodes_b: Vec<[f64; 3]> = nodes.iter().map(|p| rot(&q, *p)).collect();
            let gb = grid_of(&nodes_b, n);
            let cells_b: Vec<Prim3> = cells
                .iter()
                .map(|c| {
                    let v = rot(&q, [c[1], c[2], c[3]]);
                    [c[0], v[0], v[1], v[2], c[4]]
                })
                .collect();
            let wb = padded(&gb, &solid, &cells_b);
            let (gradb, mub, fvib, fvjb, fvkb) = run_passes(&gb, &solid, &wb, &mu_t, &wall);
            for (a, b) in mu.iter().zip(&mub) {
                assert_eq!(a, b);
            }
            let _ = (&grad, &gradb);
            for (fa, fb) in [(&fvi, &fvib), (&fvj, &fvjb), (&fvk, &fvkb)] {
                for (a, b) in fa.iter().zip(fb.iter()) {
                    let want = rot(&q, [a[1], a[2], a[3]]);
                    close(b[1], want[0], 1e-11, "momentum x");
                    close(b[2], want[1], 1e-11, "momentum y");
                    close(b[3], want[2], 1e-11, "momentum z");
                    close(b[4], a[4], 1e-11, "energy");
                    assert_eq!(b[0], 0.0);
                }
            }
        }
    }

    /// Mirror about the plane `x = 0` realised as an index reversal `i -> ni - i` of the same cells (so the grids stay right-handed): the face normals of
    /// the i-family point the other way, so their flux reverses (`Fv.n` is linear in `n`), the rest is the mirror image.
    #[test]
    fn index_reversal_mirror_reverses_the_normal_flux() {
        let mut rng = Rng::new(31);
        let (nodes, n, cells, mu_t, wall) = random_case(&mut rng);
        let g = grid_of(&nodes, n);
        // mirrored copy: x -> -x and cell column i -> ni - 1 - i, solid box mirrored too
        let nodes_b: Vec<[f64; 3]> = (0..nodes.len())
            .map(|q| {
                let (i, rest) = (q % (n[0] + 1), q / (n[0] + 1));
                let p = nodes[rest * (n[0] + 1) + (n[0] - i)];
                [-p[0], p[1], p[2]]
            })
            .collect();
        let mut gb = Grid3::try_from_nodes(
            n[0],
            n[1],
            n[2],
            nodes_b.iter().map(|p| p[0]).collect(),
            nodes_b.iter().map(|p| p[1]).collect(),
            nodes_b.iter().map(|p| p[2]).collect(),
        )
        .unwrap();
        set_solids(&mut gb, vec![SolidBox { i: (n[0] - 4)..(n[0] - 2), j: 1..3, k: 1..2 }]);
        let solid = solid_mask(&g);
        let solid_b = solid_mask(&gb);
        let mirror_cell = |c: usize| (c / n[0]) * n[0] + (n[0] - 1 - c % n[0]);
        let cells_b: Vec<Prim3> = (0..cells.len())
            .map(|c| {
                let s = cells[mirror_cell(c)];
                [s[0], -s[1], s[2], s[3], s[4]]
            })
            .collect();
        let mu_t_b: Vec<f64> = (0..cells.len()).map(|c| mu_t[mirror_cell(c)]).collect();
        let wall_b: Vec<[f64; 3]> = (0..cells.len()).map(|c| wall[mirror_cell(c)]).collect();
        let w = padded(&g, &solid, &cells);
        let wb = padded(&gb, &solid_b, &cells_b);
        let (_, _, fvi, fvj, fvk) = run_passes(&g, &solid, &w, &mu_t, &wall);
        let (_, _, fvib, fvjb, fvkb) = run_passes(&gb, &solid_b, &wb, &mu_t_b, &wall_b);
        let (ni, nj, nk) = (n[0], n[1], n[2]);
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..=ni {
                    let (a, b) = (fvi[(k * nj + j) * (ni + 1) + i], fvib[(k * nj + j) * (ni + 1) + (ni - i)]);
                    // momentum: the vector is mirrored (x flips) and the normal flips
                    for (m, f) in [(1, 1.0), (2, -1.0), (3, -1.0)] {
                        close(b[m], f * a[m], 1e-11, &format!("i-face ({i},{j},{k}) comp {m}"));
                    }
                    close(b[4], -a[4], 1e-11, "i-face energy");
                }
            }
            for j in 0..=nj {
                for i in 0..ni {
                    let (a, b) = (fvj[(k * (nj + 1) + j) * ni + i], fvjb[(k * (nj + 1) + j) * ni + (ni - 1 - i)]);
                    close(b[1], -a[1], 1e-11, "j-face momentum x");
                    close(b[2], a[2], 1e-11, "j-face momentum y");
                    close(b[3], a[3], 1e-11, "j-face momentum z");
                    close(b[4], a[4], 1e-11, "j-face energy");
                }
            }
        }
        for k in 0..=nk {
            for j in 0..nj {
                for i in 0..ni {
                    let (a, b) = (fvk[(k * nj + j) * ni + i], fvkb[(k * nj + j) * ni + (ni - 1 - i)]);
                    close(b[1], -a[1], 1e-11, "k-face momentum x");
                    close(b[2], a[2], 1e-11, "k-face momentum y");
                    close(b[3], a[3], 1e-11, "k-face momentum z");
                    close(b[4], a[4], 1e-11, "k-face energy");
                }
            }
        }
    }

    /// `reflect_grad` is the gradient of the mirror-image field: for a linear field `phi(x) = G x` the field `R phi(R x)` has the gradient `R G R` (velocity) and
    /// `R grad T` (temperature), reflecting twice is the identity, and a gradient that is already symmetric about the plane is unchanged.
    #[test]
    fn reflected_gradient_is_the_gradient_of_the_mirror_image() {
        let mut rng = Rng::new(5);
        for _ in 0..50 {
            let n = {
                let v = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
                let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
                [v[0] / l, v[1] / l, v[2] / l]
            };
            let g: Grad3 = std::array::from_fn(|_| rng.range(-1.0, 1.0));
            let rn = |x: [f64; 3]| {
                let d = x[0] * n[0] + x[1] * n[1] + x[2] * n[2];
                [x[0] - 2.0 * d * n[0], x[1] - 2.0 * d * n[1], x[2] - 2.0 * d * n[2]]
            };
            // phi_g(x) = R phi(R x) for the linear phi: velocity u_a = sum_b G[a][b] x_b, T = grad T . x
            let phi = |x: [f64; 3]| -> [f64; 4] { std::array::from_fn(|m| (0..3).map(|b| g[3 * m + b] * x[b]).sum()) };
            let phi_g = |x: [f64; 3]| -> [f64; 4] {
                let p = phi(rn(x));
                let v = rn([p[0], p[1], p[2]]);
                [v[0], v[1], v[2], p[3]]
            };
            let rg = reflect_grad(&g, n);
            let h = 0.37;
            for b in 0..3 {
                let mut e = [0.0; 3];
                e[b] = h;
                let (p1, p0) = (phi_g(e), phi_g([0.0; 3]));
                for m in 0..4 {
                    close((p1[m] - p0[m]) / h, rg[3 * m + b], 1e-13, "gradient of the mirror image");
                }
            }
            let back = reflect_grad(&rg, n);
            for q in 0..12 {
                close(back[q], g[q], 1e-13, "reflect twice");
            }
        }
    }

    /// A symmetry plane carries no shear and no heat flux: with the sides flagged as symmetry, the box `k = 0` plane of a flow whose normal velocity component
    /// varies along the plane (and with a temperature gradient along it) has only the normal stress `tau_nn` on its momentum (`x` and `y` momentum flux 0, energy
    /// flux 0); unflagged, the unreflected ghost gradient leaves the shear `mu dw/dx` there.
    #[test]
    fn symmetry_plane_carries_normal_stress_only() {
        let h = [0.1, 0.1, 0.1];
        let n = [5, 4, 4];
        let g = box_grid(n, h);
        let solid = vec![false; g.ni * g.nj * g.nk];
        // mirror-symmetric about z = 0: u, v even in z, w odd, T even; the cell centres are at z = (k + 1/2) h with the plane at z = 0 as k = 0 face
        let state = |x: [f64; 3]| -> Prim3 {
            [1.0, 0.3 + 0.4 * x[0] + 0.2 * x[2] * x[2], 0.1 * x[1], 0.5 * x[2] * (1.0 + x[0]), (1.0 + 0.3 * x[0] + 0.1 * x[2] * x[2]) / GAMMA]
        };
        let (ni, nj, nk) = (n[0], n[1], n[2]);
        let ng = NG as isize;
        let mut w = vec![[0.0; 5]; (ni + 2 * NG) * (nj + 2 * NG) * (nk + 2 * NG)];
        for k in -ng..nk as isize + ng {
            for j in -ng..nj as isize + ng {
                for i in -ng..ni as isize + ng {
                    w[pidx3(ni, nj, nk, i, j, k)] = state([(i as f64 + 0.5) * h[0], (j as f64 + 0.5) * h[1], (k as f64 + 0.5) * h[2]]);
                }
            }
        }
        let mu_t = vec![0.0; solid.len()];
        let sym = [false, false, false, false, true, false];
        let (_, _, _, _, fvk) = run_passes_sym(&g, &solid, &w, &mu_t, &[], &sym);
        let (_, _, _, _, fvk_plain) = run_passes_sym(&g, &solid, &w, &mu_t, &[], &[false; 6]);
        let mut shear_plain = 0.0f64;
        for j in 0..nj {
            for i in 0..ni {
                let f = fvk[g.cell(i, j, 0)];
                assert!(f[1].abs() < 1e-15 && f[2].abs() < 1e-15 && f[4].abs() < 1e-15, "plane k = 0 face ({i},{j}): {f:?}");
                assert!(f[3].abs() > 1e-6, "the normal stress is there: {f:?}");
                shear_plain = shear_plain.max(fvk_plain[g.cell(i, j, 0)][1].abs());
            }
        }
        assert!(shear_plain > 1e-7, "the unflagged plane has a shear of {shear_plain:e}");
    }

    /// The adiabatic wall carries no heat flux on a sheared grid. The columns are sheared (`x += 0.4 y`) so the centre of a wall cell is not above the centroid
    /// of its wall face: the ghost sits at the mirror image of the centre in the plane of the face, so the compact gradient is along the wall normal and the
    /// tangential temperature gradient never leaks into the normal flux (`e` along the line from centre to the point-mirrored ghost would leave `~ 0.4 dT/dx`).
    #[test]
    fn adiabatic_wall_has_no_heat_flux_on_a_sheared_grid() {
        let (ni, nj, nk) = (6, 5, 3);
        let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
        for k in 0..=nk {
            for j in 0..=nj {
                for i in 0..=ni {
                    x.push(0.1 * i as f64 + 0.04 * j as f64);
                    y.push(0.05 * j as f64);
                    z.push(0.1 * k as f64);
                }
            }
        }
        let mut g = Grid3::try_from_nodes(ni, nj, nk, x, y, z).unwrap();
        set_solids(&mut g, vec![SolidBox { i: 0..ni, j: 0..1, k: 0..nk }]);
        let solid = solid_mask(&g);
        // T varies along the wall (x) and away from it; rho uniform; no velocity
        let cells: Vec<Prim3> = (0..ni * nj * nk).map(|c| [1.0, 0.0, 0.0, 0.0, (1.0 + 0.8 * g.xc[c][0] + 0.5 * g.xc[c][1]) / GAMMA]).collect();
        let w = padded(&g, &solid, &cells);
        let mu_t = vec![0.0; solid.len()];
        let (_, _, _, fvj, _) = run_passes(&g, &solid, &w, &mu_t, &[]);
        for k in 0..nk {
            for i in 0..ni {
                let (q, face) = face_of(&g, 1, i, 1, k);
                assert!(face.n[0].abs() < 1e-12 && (face.n[1] - 1.0).abs() < 1e-12, "the wall face is horizontal: {:?}", face.n);
                assert!(fvj[q][4].abs() < 1e-15, "heat flux through the adiabatic wall: {:e}", fvj[q][4]);
            }
        }
    }

    /// `diffusion_rate` and the face `lambda` on a box of cell sizes `h`: `2 lv A_d^2 / V = 2 lv h_a h_b / h_d` per direction `d` (the face area `A_d` weighs the
    /// rate like the convective `(|u| + c) A_d`; equal to the 2D `viscous::diffusion_rate` with the face length -> the face area and the cell area -> the volume),
    /// `gamma (mu / Pr + mu_t / Pr_t) / (rho d)` per face, 0 on the faces that carry no flux (the faces of a solid cell, a degenerate face).
    #[test]
    fn diffusion_rates_and_face_lambdas_of_a_box() {
        let h = [0.1, 0.2, 0.3];
        let n = [4, 4, 4];
        let mut g = box_grid(n, h);
        set_solids(&mut g, vec![SolidBox { i: 1..2, j: 1..2, k: 1..2 }]);
        let solid = solid_mask(&g);
        let (rho, p) = (1.3, 0.9);
        let cells = vec![[rho, 0.2, 0.0, 0.0, p]; solid.len()];
        let w = padded(&g, &solid, &cells);
        let mu: Vec<f64> = vec![VISC.mu(temperature3(&cells[0])); solid.len()];
        let mu_t: Vec<f64> = (0..solid.len()).map(|c| 1e-4 * (1 + c % 3) as f64).collect();
        let dirs = face_dirs(&g, &solid, &[false; 6]);
        let c = g.cell(2, 2, 2);
        let rates = diffusion_rates(&g, 2, 2, 2, mu[c], mu_t[c], rho);
        let lv = GAMMA * (mu[c] / PRANDTL + mu_t[c] / PRANDTL_T) / rho;
        for (dir, rate) in rates.iter().enumerate() {
            close(*rate, 2.0 * lv * h[(dir + 1) % 3] * h[(dir + 2) % 3] / h[dir], 1e-12, "diffusion rate");
            let (_, f) = face_of(&g, dir, 2, 2, 2);
            close(*rate, crate::viscous::diffusion_rate(mu[c], mu_t[c], rho, f.area, g.vol[c]), 1e-12, "the 2D formula");
        }
        let (mut li, mut lj, mut lk) = (vec![0.0; g.iface.len()], vec![0.0; g.jface.len()], vec![0.0; g.kface.len()]);
        lambda_pass(&g, &solid, &w, &dirs, &mu, &mu_t, &[], None, &mut li, &mut lj, &mut lk);
        // interior face between two fluid cells: d = h
        let (q, _) = face_of(&g, 0, 3, 2, 2);
        let (mu_f, mut_f) = (mu[c], 0.5 * (mu_t[g.cell(2, 2, 2)] + mu_t[g.cell(3, 2, 2)]));
        close(li[q], GAMMA * (mu_f / PRANDTL + mut_f / PRANDTL_T) / (rho * h[0]), 1e-12, "interior face lambda");
        // a wall face (the solid cell (1,1,1) below the fluid cell (1,2,1) in j) has d = h as well and takes the wall ghost: the same mu, mu_t of the fluid cell
        // with the face eddy viscosity 0 (smooth wall: `wall` empty)
        let (qw, _) = face_of(&g, 1, 1, 2, 1);
        close(lj[qw], GAMMA * mu[c] / PRANDTL / (rho * h[1]), 1e-12, "wall face lambda (mu_t,w = 0)");
        assert!(dirs.j[qw].noslip);
        // a face between two solid cells carries nothing; the face between solid (1,1,1) and the fluid (0,1,1) is a wall
        let (q_in, _) = face_of(&g, 0, 1, 1, 1);
        assert!(dirs.i[q_in].noslip && li[q_in] > 0.0);
        // lambda_pass and the single-face function agree, `face_lambdas` returns the lower and upper face of a cell
        let (lo, hi) = face_lambdas(&g, &solid, &w, &dirs, &mu, &mu_t, &[], None, 0, 2, 2, 2);
        assert_eq!((lo, hi), (li[face_of(&g, 0, 2, 2, 2).0], li[face_of(&g, 0, 3, 2, 2).0]));
    }
}

/// Wedge parity: the 3D kernels on a one-cell wedge `revolve(g2, 1, [0, dtheta])` against the validated 2D passes of [`crate::viscous`] (with its hoop-stress
/// source) on the same meridian grid and the same smooth axisymmetric state (`wedge::smooth_state`), laminar, with and without a body (a solid cell block with
/// a no-slip surface and base).
#[cfg(test)]
mod parity {
    use super::tests::run_passes_sym;
    use super::wedge::{analytic_viscous_residual, col_x, smooth_state, viscous_residual_2d, wedge, Wedge};
    use super::*;

    /// The wedge sides: `k = 0` and `k = nk` are symmetry planes.
    const SYM: SymSides = [false, false, false, false, true, true];

    /// The 3D viscous residual of a wedge case in the meridian frame `[x-momentum, radial momentum, energy]` per cell (radial = `v sin + w cos` at the mid
    /// azimuth), the largest azimuthal momentum and mass residual relative to the largest x-momentum residual.
    pub(super) struct Run3 {
        pub r: Vec<[f64; 3]>,
        pub azimuthal: f64,
        pub mass: f64,
    }

    pub(super) fn run3(wd: &Wedge) -> Run3 {
        let n = wd.g2.ni * wd.g2.nj;
        let mu_t = vec![0.0; n];
        let (_, _, fvi, fvj, fvk) = run_passes_sym(&wd.g3, &wd.solid, &wd.w3, &mu_t, &[], &SYM);
        let mut res = vec![[0.0; 5]; n];
        add_viscous_residual(&wd.g3, &wd.solid, &fvi, &fvj, &fvk, &mut res);
        let (s, c) = wd.theta_m.sin_cos();
        let scale = res.iter().fold(0.0f64, |m, r| m.max(r[1].abs()));
        let azimuthal = res.iter().fold(0.0f64, |m, r| m.max((r[2] * c - r[3] * s).abs())) / scale;
        let mass = res.iter().fold(0.0f64, |m, r| m.max(r[0].abs())) / scale;
        Run3 { r: res.iter().map(|r| [r[1], r[2] * s + r[3] * c, r[4]]).collect(), azimuthal, mass }
    }

    pub(super) fn run2(wd: &Wedge) -> Vec<[f64; 3]> {
        viscous_residual_2d(wd, &vec![0.0; wd.g2.ni * wd.g2.nj]).iter().map(|r| [r[1], r[2], r[3]]).collect()
    }

    /// Cells compared: every fluid cell (`all`), or those at least 2 cells away from the domain boundaries and, where the axis is a boundary of the fluid
    /// (no body, or the wake behind the base), more than 8 cells away from it (`inner`: both schemes have an `O(h / r)` error in the viscous radial momentum
    /// of the first rows beside the axis, from the cancellation of the `1 / r` terms).
    pub(super) fn mask(wd: &Wedge, inner: bool) -> Vec<bool> {
        let (ni, nj) = (wd.g2.ni, wd.g2.nj);
        (0..ni * nj)
            .map(|c| {
                let (i, j) = (c % ni, c / ni);
                let axis = wd.blk.is_none_or(|b| i >= b.i_end);
                !wd.solid[c] && (!inner || (i >= 2 && i + 2 < ni && j + 2 < nj && !(axis && j < 8)))
            })
            .collect()
    }

    /// `(L2, Linf)` per component of `a - b` relative to the same norm of `b` over the cells of `keep`.
    pub(super) fn rel(a: &[[f64; 3]], b: &[[f64; 3]], keep: &[bool]) -> ([f64; 3], [f64; 3]) {
        let (mut d2, mut n2, mut dm, mut nm) = ([0.0f64; 3], [0.0f64; 3], [0.0f64; 3], [0.0f64; 3]);
        for c in (0..a.len()).filter(|&c| keep[c]) {
            for m in 0..3 {
                d2[m] += (a[c][m] - b[c][m]).powi(2);
                n2[m] += b[c][m] * b[c][m];
                dm[m] = dm[m].max((a[c][m] - b[c][m]).abs());
                nm[m] = nm[m].max(b[c][m].abs());
            }
        }
        ([0, 1, 2].map(|m| (d2[m] / n2[m]).sqrt()), [0, 1, 2].map(|m| dm[m] / nm[m]))
    }

    pub(super) fn build(ni: usize, nj: usize, wall: bool, deg: f64, matched: bool) -> Wedge {
        wedge(ni, nj, wall, deg.to_radians(), matched, &smooth_state(wall.then(|| col_x(ni / 2, ni))))
    }

    /// Table of the measured differences (run with `--ignored --no-capture`): per grid, angle and wedge variant, the `L2` and `Linf` relative differences of the 3D
    /// residual to the 2D one and to the same wedge at 0.05 degrees (the wedge-angle error alone), over all fluid cells and over the `inner` ones.
    #[test]
    #[ignore = "prints the parity table"]
    fn report() {
        for matched in [false, true] {
            for wall in [false, true] {
                for (ni, nj) in [(48usize, 36usize), (96, 72)] {
                    let lim = run3(&build(ni, nj, wall, 0.05, matched));
                    for deg in [12.0, 6.0, 2.0, 1.0] {
                        let wd = build(ni, nj, wall, deg, matched);
                        let r3 = run3(&wd);
                        let r2 = run2(&wd);
                        let fmt = |n: ([f64; 3], [f64; 3])| {
                            format!("l2 [{:.2e} {:.2e} {:.2e}] linf [{:.2e} {:.2e} {:.2e}]", n.0[0], n.0[1], n.0[2], n.1[0], n.1[1], n.1[2])
                        };
                        println!("matched {matched:5} wall {wall:5} {ni}x{nj} {deg:4} deg  azimuthal {:.1e} mass {:.1e}", r3.azimuthal, r3.mass);
                        println!("    3D-2D inner {}", fmt(rel(&r3.r, &r2, &mask(&wd, true))));
                        println!("    3D-2D all   {}", fmt(rel(&r3.r, &r2, &mask(&wd, false))));
                        println!("    3D-3D(0.05 deg) all   {}", fmt(rel(&r3.r, &lim.r, &mask(&wd, false))));
                        println!("    3D-3D(0.05 deg) inner {}", fmt(rel(&r3.r, &lim.r, &mask(&wd, true))));
                    }
                }
            }
        }
    }

    /// The error of a wedge of opening angle `dtheta` against the same wedge at 0.05 degrees (the wedge-angle error alone, free of the difference of the two
    /// discretisations) falls by 4 per halving and 9 per third of the angle: `err(12) / err(6) = 4`, `err(6) / err(2) = 9`, `err(2) / err(1) = 4`, and is
    /// at most `5e-4` at 2 degrees (relative `L2` norm of every residual component over the `inner` cells), with and without a body, for the wedge with the
    /// flat faces cross the mid-plane at the radii of the 2D grid (`matched`) and, without a body, for the plain `grid3::revolve` wedge.
    #[test]
    fn wedge_angle_error_falls_with_the_square_of_the_angle() {
        for (wall, matched) in [(false, true), (true, true), (false, false)] {
            let (ni, nj) = (48, 36);
            let lim = run3(&build(ni, nj, wall, 0.05, matched));
            let err = |deg: f64| {
                let wd = build(ni, nj, wall, deg, matched);
                rel(&run3(&wd).r, &lim.r, &mask(&wd, true)).0
            };
            let (e12, e6, e2, e1) = (err(12.0), err(6.0), err(2.0), err(1.0));
            for m in 0..3 {
                let tag = format!("wall {wall} matched {matched} component {m}: errors {:e} {:e} {:e} {:e}", e12[m], e6[m], e2[m], e1[m]);
                assert!((3.7..4.3).contains(&(e12[m] / e6[m])), "12 -> 6 deg: {tag}");
                assert!((8.0..10.0).contains(&(e6[m] / e2[m])), "6 -> 2 deg: {tag}");
                assert!((3.7..4.3).contains(&(e2[m] / e1[m])), "2 -> 1 deg: {tag}");
                assert!(e2[m] <= 5e-4, "2 deg: {tag}");
            }
        }
    }

    /// The wedge reproduces the viscous residual of the 2D axisymmetric solver including its hoop-stress source: at 2 degrees the relative `L2` difference of the
    /// `inner` cells is below the bounds for `[x-momentum, radial momentum, energy]`. This is the difference of two consistent second-order discretisations, not
    /// an angle effect (it is the same at 1 degree, see `report`): it falls with the grid for the axial momentum and the energy (measured `L2` at 48x36 /
    /// 96x72, no body: `[1.5e-4 6.3e-4 8.5e-5]` / `[8.0e-5 9.0e-4 3.9e-5]`; with a body: `[1.0e-3 2.1e-2 1.3e-4]` / `[4.5e-4 1.3e-2 3.3e-5]`); the radial momentum
    /// keeps the `O(h / r)` difference of the first cells beside the axis / the body surface (the gradient of a cell next to the boundary is one-sided there, the
    /// 3D one weights the faces by their areas, the 2D one does not).
    #[test]
    fn wedge_matches_the_2d_viscous_residual() {
        let bound = |wall: bool, matched: bool| match (wall, matched) {
            (true, _) => [2.0e-3, 3.5e-2, 3.0e-4],
            (false, true) => [3.0e-4, 1.5e-3, 2.0e-4],
            (false, false) => [3.0e-4, 1.5e-3, 6.0e-4],
        };
        let mut errs = [[0.0; 3]; 2];
        for wall in [false, true] {
            for (g, (ni, nj)) in [(48usize, 36usize), (96, 72)].into_iter().enumerate() {
                for matched in [true, false] {
                    if wall && !matched {
                        continue; // the plain revolve puts the wall `dtheta^2 / 8` inside the cylinder on which the state of the 2D grid vanishes
                    }
                    let wd = build(ni, nj, wall, 2.0, matched);
                    let e = rel(&run3(&wd).r, &run2(&wd), &mask(&wd, true)).0;
                    for m in 0..3 {
                        assert!(
                            e[m] <= bound(wall, matched)[m],
                            "wall {wall} matched {matched} {ni}x{nj} component {m}: {:e} > {:e}",
                            e[m],
                            bound(wall, matched)[m]
                        );
                    }
                    if matched {
                        errs[g] = e;
                    }
                }
            }
            // the axial momentum and the energy fall with the grid
            for m in [0, 2] {
                assert!(errs[1][m] < 0.6 * errs[0][m], "wall {wall} component {m}: {:?} -> {:?}", errs[0], errs[1]);
            }
        }
    }

    /// The 3D wedge approaches the continuous axisymmetric viscous operator (`wedge::analytic_viscous_residual` of the smooth state at the cell centres, by
    /// high-order differentiation) with the grid: with no body, at 1 degree, the `inner` cells of `[x-momentum, radial momentum, energy]` converge at second
    /// order for the axial momentum and the energy (ratio `>= 3` per halving of `h`), and the radial momentum stays within `3e-3` (`O(h / r)` beside the axis).
    #[test]
    fn wedge_residual_converges_to_the_continuous_operator() {
        let mut errs = vec![];
        for (ni, nj) in [(48usize, 36usize), (96, 72)] {
            let wd = build(ni, nj, false, 1.0, true);
            let f = smooth_state(None);
            let an: Vec<[f64; 3]> = (0..ni * nj).map(|c| analytic_viscous_residual(&f, wd.g3.xc[c][0], wd.g3.xc[c][1].hypot(wd.g3.xc[c][2]))).collect();
            errs.push(rel(&run3(&wd).r, &an, &mask(&wd, true)).0);
        }
        for m in [0, 2] {
            assert!(errs[0][m] > 3.0 * errs[1][m], "component {m}: {:e} -> {:e}", errs[0][m], errs[1][m]);
        }
        assert!(errs[1][1] < 3e-3, "radial momentum {:e}", errs[1][1]);
    }

    /// An axisymmetric state has no azimuthal momentum and no mass flux in the viscous residual of the wedge, to roundoff (the sum of the two symmetric `k`
    /// faces), at every angle.
    #[test]
    fn wedge_residual_has_no_azimuthal_momentum_or_mass() {
        for wall in [false, true] {
            for deg in [12.0, 2.0] {
                let r = run3(&build(48, 36, wall, deg, true));
                assert!(r.azimuthal < 1e-9 && r.mass == 0.0, "wall {wall} {deg} deg: azimuthal {:e} mass {:e}", r.azimuthal, r.mass);
            }
        }
    }
}
