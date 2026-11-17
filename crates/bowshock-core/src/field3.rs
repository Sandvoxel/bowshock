//! The 3D flow field of a point: the state the 3D solver iterates, on the grid its forces are computed on, with the wall faces of
//! everything that is solid, for a viewer that knows nothing of the solver's topology.
//!
//! The 3D counterpart of [`Field`](crate::Field), plain data with an `f32` payload (a snapshot is for looking at; forces are integrated from the
//! solver's own `f64` state). A 2 million cell field is 40 MB of state and 24 MB of nodes.
//!
//! # The grid
//!
//! Structured and body-fitted: `ni` cells downstream, `nj` outwards, `nk` around the axis. Node `(i, j, k)` is stored at
//! `(k (nj + 1) + j)(ni + 1) + i`, cell `(i, j, k)` at `(k nj + j) ni + i` (the layouts of `bowshock_cfd::grid3::Grid3`). Nodes are in
//! metres: `x` runs downstream from the nose tip and the node is `(x, r sin(theta), r cos(theta))`, so the pitch plane is `y = 0`,
//! `theta = 0` points to `+z` and `theta = pi` to `-z`. The air moves towards `+z`: at a positive angle of attack the `+z` side
//! (`theta = 0`) is the lee side and the `-z` side the windward one.
//!
//! # The state
//!
//! Per cell the primitive state `[rho, u, v, w, p]` in the solver's units: density over the free-stream density, the Cartesian
//! velocity components (along `x`, `y`, `z`) over the free-stream speed of sound, pressure over `rho_inf c_inf^2`. The free stream is
//! [`Field3::freestream`]. Solid cells (fins, the body interior) hold the free stream and are not part of the flow
//! ([`Field3::is_fluid`]). [`Field3::eddy_ratio`] is the ratio of eddy to molecular viscosity per cell for a RANS run, empty otherwise.
//!
//! # The walls
//!
//! [`Field3::walls`] lists every wall face of the solved domain as a quad of node indices, wound counter-clockwise seen from the fluid
//! (the normal `(p2 - p0) x (p3 - p1)` points into the flow), with the fluid cell next to it, the wall pressure coefficient (the
//! two-cell linear extrapolation the force integration uses, so that `sum -cp A n` reproduces the reported pressure force), the skin
//! friction coefficient vector (zero for an Euler run) and the [`Part`] of the vehicle it belongs to.
//!
//! # The symmetry
//!
//! The solved domain is a range of azimuth. [`Symmetry`] says how it maps onto the whole flow and [`Symmetry::tiles`] lists the
//! orthogonal maps that tile the full circle with it.

use std::f32::consts::TAU;

/// Ratio of specific heats of the gas the solver uses (air).
pub use crate::field::GAMMA;

/// The part of the vehicle a wall face belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Part {
    /// The nose (up to the end of the nose cone or ogive).
    Nose,
    /// The body behind the nose, up to the base.
    Body,
    /// The base face.
    Base,
    /// A fin set (the index counts the sets).
    Fin(u8),
    /// Another appendage: a pod, lug or rail button (the index counts them).
    Appendage(u8),
    /// The motor jet's nozzle exit.
    Jet,
    /// Anything else (the sting of the sting grid).
    Other,
}

/// One wall face of the solved domain.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WallFace {
    /// The four nodes (indices into [`Field3::nodes`]), counter-clockwise seen from the fluid.
    pub nodes: [u32; 4],
    /// The fluid cell next to the face.
    pub cell: u32,
    /// The part of the vehicle.
    pub part: Part,
    /// Pressure coefficient `(p_w - p_inf) / (rho_inf |V_inf|^2 / 2)`.
    pub cp: f32,
    /// Skin-friction coefficient vector (over `rho_inf |V_inf|^2 / 2`, Cartesian components); zero for an Euler run.
    pub cf: [f32; 3],
}

/// How the solved range of azimuth `theta0..theta1` (radians) maps onto the whole flow.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Symmetry {
    /// The whole circle was solved.
    Full,
    /// The flow is mirror-symmetric about both bounding half-planes: the whole flow is `2 pi / (theta1 - theta0)` copies, every other one
    /// mirrored (the width is `pi / m`: the half domain at angle of attack is `0..pi`, the sector of `n` fins at zero incidence `pi / n` wide).
    Mirror {
        /// First azimuth.
        theta0: f32,
        /// Last azimuth.
        theta1: f32,
    },
    /// The flow repeats with the sector: the whole flow is `2 pi / (theta1 - theta0)` rotated copies.
    Periodic {
        /// First azimuth.
        theta0: f32,
        /// Last azimuth.
        theta1: f32,
    },
    /// The flow is axisymmetric and the domain is a one-cell wedge of it: any rotation about the axis is a symmetry.
    Axisymmetric {
        /// First azimuth.
        theta0: f32,
        /// Last azimuth.
        theta1: f32,
    },
}

/// One copy of the solved domain in the whole flow: the orthogonal map `m` on `(x, y, z)` column vectors. `mirrored` (determinant
/// -1) means the copy's triangles must be wound the other way round.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tile {
    /// Row-major 3 x 3 matrix.
    pub m: [[f32; 3]; 3],
    /// The map is a reflection.
    pub mirrored: bool,
}

impl Tile {
    /// The identity.
    pub const IDENTITY: Tile = Tile { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], mirrored: false };

    /// Rotation about the `x` axis by `phi` towards increasing azimuth (`theta -> theta + phi`).
    pub fn rotation(phi: f32) -> Tile {
        let (s, c) = phi.sin_cos();
        Tile { m: [[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]], mirrored: false }
    }

    /// Reflection about the half-plane of azimuth `theta` (`theta' -> 2 theta - theta'`).
    pub fn reflection(theta: f32) -> Tile {
        let (s, c) = (2.0 * theta).sin_cos();
        Tile { m: [[1.0, 0.0, 0.0], [0.0, -c, s], [0.0, s, c]], mirrored: true }
    }

    /// `self` after `first`: the map `p -> self(first(p))`.
    pub fn after(&self, first: &Tile) -> Tile {
        let mut m = [[0.0; 3]; 3];
        for (r, row) in m.iter_mut().enumerate() {
            for (c, v) in row.iter_mut().enumerate() {
                *v = (0..3).map(|k| self.m[r][k] * first.m[k][c]).sum();
            }
        }
        Tile { m, mirrored: self.mirrored != first.mirrored }
    }

    /// The image of the point (or vector) `p`.
    pub fn apply(&self, p: [f32; 3]) -> [f32; 3] {
        [0, 1, 2].map(|r| self.m[r][0] * p[0] + self.m[r][1] * p[1] + self.m[r][2] * p[2])
    }

    /// The inverse map (the transpose: the matrix is orthogonal).
    pub fn inverse(&self) -> Tile {
        let m = self.m;
        Tile { m: [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]], mirrored: self.mirrored }
    }
}

impl Symmetry {
    /// The solved range of azimuth; `Full` is `0..2 pi`.
    pub fn theta_range(&self) -> (f32, f32) {
        match *self {
            Symmetry::Full => (0.0, TAU),
            Symmetry::Mirror { theta0, theta1 } | Symmetry::Periodic { theta0, theta1 } | Symmetry::Axisymmetric { theta0, theta1 } => (theta0, theta1),
        }
    }

    /// The copies that tile the full circle, in order of azimuth: copy `m` covers `theta0 + m w .. theta0 + (m + 1) w` (`w` the width
    /// of the solved range). The first is the identity. For `Mirror`, odd copies are the solved domain reflected about the half-plane
    /// `theta0` and then rotated by `(m + 1) w`; `Periodic` rotates by `m w`; `Axisymmetric` rotates by `2 pi m / n` with `n` the whole number of wedges
    /// that fill the circle.
    pub fn tiles(&self) -> Vec<Tile> {
        let (t0, t1) = self.theta_range();
        let w = (t1 - t0).max(1e-6);
        let n = ((TAU / w).round() as usize).max(1);
        match *self {
            Symmetry::Full => vec![Tile::IDENTITY],
            Symmetry::Periodic { .. } => (0..n).map(|m| Tile::rotation(m as f32 * w)).collect(),
            Symmetry::Axisymmetric { .. } => (0..n).map(|m| Tile::rotation(TAU * m as f32 / n as f32)).collect(),
            Symmetry::Mirror { theta0, .. } => (0..n)
                .map(|m| if m % 2 == 0 { Tile::rotation(m as f32 * w) } else { Tile::rotation((m + 1) as f32 * w).after(&Tile::reflection(theta0)) })
                .collect(),
        }
    }
}

/// A snapshot of the 3D flow of one run (see the [module documentation](self) for the layout and the units).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Field3 {
    /// Free-stream Mach number of the run.
    pub mach: f64,
    /// Angle of attack (degrees); the air moves towards `+z`.
    pub alpha_deg: f64,
    /// Solver iterations done when the snapshot was taken.
    pub iteration: usize,
    /// The free stream `[rho, u, v, w, p]` in the units of [`Field3::prim`].
    pub freestream: [f64; 5],
    /// Cells along `i` (downstream).
    pub ni: usize,
    /// Cells along `j` (away from the axis).
    pub nj: usize,
    /// Cells along `k` (around the axis).
    pub nk: usize,
    /// Hash of what a viewer builds its mesh from (`nodes`, `solid`, the topology of `walls`): equal for the snapshots of one run, so a
    /// viewer can keep what it built.
    pub mesh_id: u64,
    /// Node coordinates in metres, `(ni + 1)(nj + 1)(nk + 1)`, index `(k (nj + 1) + j)(ni + 1) + i`.
    pub nodes: Vec<[f32; 3]>,
    /// Per cell: the cell is solid (a fin, the body), not part of the flow.
    pub solid: Vec<bool>,
    /// Per cell: `[rho, u, v, w, p]` (solid cells hold the free stream).
    pub prim: Vec<[f32; 5]>,
    /// Per cell: eddy viscosity over molecular viscosity; empty unless the run is RANS.
    pub eddy_ratio: Vec<f32>,
    /// Every wall face of the solved domain.
    pub walls: Vec<WallFace>,
    /// Azimuth (radians) of the mid-plane of each fin inside the solved range; empty without fins.
    pub fin_theta: Vec<f32>,
    /// How the solved domain maps onto the whole flow.
    pub symmetry: Symmetry,
}

impl Field3 {
    /// Number of cells, `ni nj nk`.
    pub fn cells(&self) -> usize {
        self.ni * self.nj * self.nk
    }

    /// Index of cell `(i, j, k)`.
    pub fn cell_index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.nj + j) * self.ni + i
    }

    /// Index of node `(i, j, k)`.
    pub fn node_index(&self, i: usize, j: usize, k: usize) -> usize {
        (k * (self.nj + 1) + j) * (self.ni + 1) + i
    }

    /// Coordinates of node `(i, j, k)`.
    pub fn node(&self, i: usize, j: usize, k: usize) -> [f32; 3] {
        self.nodes[self.node_index(i, j, k)]
    }

    /// Centre of cell `(i, j, k)`: the mean of its eight nodes.
    pub fn centre(&self, i: usize, j: usize, k: usize) -> [f32; 3] {
        let mut m = [0.0f32; 3];
        for b in 0..8 {
            let p = self.node(i + (b & 1), j + (b >> 1 & 1), k + (b >> 2 & 1));
            for a in 0..3 {
                m[a] += 0.125 * p[a];
            }
        }
        m
    }

    /// Whether cell `c` is part of the flow.
    pub fn is_fluid(&self, c: usize) -> bool {
        !self.solid[c]
    }

    /// Mach number of cell `c`.
    pub fn mach_number(&self, c: usize) -> f32 {
        let [rho, u, v, w, p] = self.prim[c];
        (u * u + v * v + w * w).sqrt() / (GAMMA as f32 * p / rho).sqrt()
    }

    /// Pressure coefficient of cell `c`: `(p - p_inf) / (rho_inf |V_inf|^2 / 2)`.
    pub fn pressure_coefficient(&self, c: usize) -> f32 {
        let [rho, u, v, w, p] = self.freestream;
        ((f64::from(self.prim[c][4]) - p) / (0.5 * rho * (u * u + v * v + w * w))) as f32
    }

    /// Mach number per cell; NaN in solid cells.
    pub fn mach_numbers(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.mach_number(c) }).collect()
    }

    /// Pressure coefficient per cell; NaN in solid cells.
    pub fn pressure_coefficients(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.pressure_coefficient(c) }).collect()
    }

    /// Magnitude of the gradient of the density (over the free-stream density) at cell `(i, j, k)`, per metre: what a schlieren picture
    /// shows. From differences of the cell-centre values along the grid directions through the cell-centre metric: central where both
    /// neighbours are flow cells, one-sided at the boundaries and beside solid cells. A direction without any neighbour (a one-cell wedge
    /// in `k`) is left out and the gradient is the one in the plane of the others. NaN in a solid cell.
    pub fn density_gradient_at(&self, i: usize, j: usize, k: usize) -> f32 {
        let c = self.cell_index(i, j, k);
        if self.solid[c] {
            return f32::NAN;
        }
        let dims = [self.ni, self.nj, self.nk];
        let at = [i, j, k];
        let mut cols: Vec<([f32; 3], f32)> = Vec::with_capacity(3);
        for d in 0..3 {
            let step = |delta: isize| -> Option<(usize, usize, usize)> {
                let v = at[d] as isize + delta;
                if v < 0 || v >= dims[d] as isize {
                    return None;
                }
                let mut p = at;
                p[d] = v as usize;
                let n = self.cell_index(p[0], p[1], p[2]);
                self.is_fluid(n).then_some((p[0], p[1], p[2]))
            };
            let (lo, hi) = (step(-1).unwrap_or((i, j, k)), step(1).unwrap_or((i, j, k)));
            if lo == hi {
                continue;
            }
            let (a, b) = (self.centre(lo.0, lo.1, lo.2), self.centre(hi.0, hi.1, hi.2));
            let (ra, rb) = (self.prim[self.cell_index(lo.0, lo.1, lo.2)][0], self.prim[self.cell_index(hi.0, hi.1, hi.2)][0]);
            cols.push(([b[0] - a[0], b[1] - a[1], b[2] - a[2]], rb - ra));
        }
        let rho_inf = self.freestream[0] as f32;
        let dot = |a: [f32; 3], b: [f32; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let g = match cols.len() {
            0 => 0.0,
            // gradient in the span of the differences: g = J (J^T J)^-1 d, solved through the Gram matrix of the columns
            1 => {
                let (a, d) = cols[0];
                let n = dot(a, a);
                if n > 0.0 {
                    (d / n.sqrt()).abs()
                } else {
                    0.0
                }
            }
            2 => {
                let ((a, da), (b, db)) = (cols[0], cols[1]);
                let (g11, g12, g22) = (dot(a, a), dot(a, b), dot(b, b));
                let det = g11 * g22 - g12 * g12;
                if det.abs() > 1e-30 {
                    let (s, t) = ((g22 * da - g12 * db) / det, (g11 * db - g12 * da) / det);
                    let v = [0, 1, 2].map(|m| s * a[m] + t * b[m]);
                    dot(v, v).sqrt()
                } else {
                    0.0
                }
            }
            _ => {
                let ((a, da), (b, db), (c3, dc)) = (cols[0], cols[1], cols[2]);
                // solve [a b c]^T g = d by Cramer's rule on the rows a, b, c
                let cross = |p: [f32; 3], q: [f32; 3]| [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
                let det = dot(a, cross(b, c3));
                if det.abs() > 1e-30 {
                    let (bc, ca, ab) = (cross(b, c3), cross(c3, a), cross(a, b));
                    let v = [0, 1, 2].map(|m| (da * bc[m] + db * ca[m] + dc * ab[m]) / det);
                    dot(v, v).sqrt()
                } else {
                    0.0
                }
            }
        };
        g / rho_inf
    }

    /// [`Field3::density_gradient_at`] for every cell; NaN in solid cells.
    pub fn density_gradient(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.cells());
        for k in 0..self.nk {
            for j in 0..self.nj {
                for i in 0..self.ni {
                    out.push(self.density_gradient_at(i, j, k));
                }
            }
        }
        out
    }

    /// Azimuth (radians, from the start of the solved range) of the middle of cell layer `k`, read from the nodes at the outer edge of the
    /// grid, where they are far from the axis.
    pub fn theta_of_layer(&self, k: usize) -> f32 {
        let at = |kk: usize| {
            let p = self.node(self.ni / 2, self.nj, kk);
            let t = p[1].atan2(p[2]);
            let t0 = self.symmetry.theta_range().0;
            if t < t0 - 1e-3 {
                t + TAU
            } else {
                t
            }
        };
        0.5 * (at(k) + at(k + 1))
    }

    /// The cell layer whose middle is nearest the azimuth `theta` (radians, any turn).
    pub fn layer_of_theta(&self, theta: f32) -> usize {
        let t0 = self.symmetry.theta_range().0;
        let t = (theta - t0).rem_euclid(TAU) + t0;
        let dist = |k: usize| {
            let d = (self.theta_of_layer(k) - t).abs();
            d.min(TAU - d)
        };
        (0..self.nk).min_by(|&a, &b| dist(a).total_cmp(&dist(b))).unwrap_or(0)
    }

    /// The bounding box `(min, max)` of the nodes, over the solved domain.
    pub fn bounds(&self) -> ([f32; 3], [f32; 3]) {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for p in &self.nodes {
            for a in 0..3 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
        (lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    /// Angle of azimuth of the image of the sector's mid-plane point under each tile.
    fn tile_azimuths(s: Symmetry) -> Vec<f32> {
        let (t0, t1) = s.theta_range();
        // a probe point inside the sector, away from its bounding planes
        let t = t0 + 0.3 * (t1 - t0);
        let p = [0.0, t.sin(), t.cos()];
        s.tiles()
            .iter()
            .map(|tile| {
                let q = tile.apply(p);
                q[1].atan2(q[2]).rem_euclid(TAU)
            })
            .collect()
    }

    fn assert_orthogonal(t: &Tile) {
        let id = t.after(&t.inverse());
        for r in 0..3 {
            for c in 0..3 {
                assert!((id.m[r][c] - if r == c { 1.0 } else { 0.0 }).abs() < 1e-5);
            }
        }
        let m = t.m;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        assert!((det - if t.mirrored { -1.0 } else { 1.0 }).abs() < 1e-5, "det {det}");
    }

    #[test]
    fn the_tiles_of_every_symmetry_cover_the_circle_once() {
        for (s, n) in [
            (Symmetry::Full, 1),
            (Symmetry::Mirror { theta0: 0.0, theta1: PI }, 2),
            (Symmetry::Mirror { theta0: 0.0, theta1: PI / 4.0 }, 8),
            (Symmetry::Mirror { theta0: 0.2, theta1: 0.2 + PI / 3.0 }, 6),
            (Symmetry::Periodic { theta0: 0.0, theta1: TAU / 3.0 }, 3),
            (Symmetry::Axisymmetric { theta0: 0.0, theta1: TAU / 72.0 }, 72),
        ] {
            let tiles = s.tiles();
            assert_eq!(tiles.len(), n, "{s:?}");
            assert_eq!(tiles[0], Tile::IDENTITY);
            tiles.iter().for_each(assert_orthogonal);
            // the mirrored flags alternate for Mirror, are absent otherwise
            for (m, t) in tiles.iter().enumerate() {
                assert_eq!(t.mirrored, matches!(s, Symmetry::Mirror { .. }) && m % 2 == 1, "{s:?} tile {m}");
            }
            // the image of a probe point in the sector falls in the tile's azimuth range, once each
            if n > 1 {
                let (t0, t1) = s.theta_range();
                let w = t1 - t0;
                for (m, th) in tile_azimuths(s).into_iter().enumerate() {
                    let local = (th - t0).rem_euclid(TAU);
                    let (lo, hi) = (m as f32 * w, (m + 1) as f32 * w);
                    assert!(local >= lo - 1e-4 && local <= hi + 1e-4, "{s:?} tile {m}: {local} not in {lo}..{hi}");
                }
            }
        }
    }

    #[test]
    fn the_half_domain_mirrors_about_the_pitch_plane() {
        let t = Symmetry::Mirror { theta0: 0.0, theta1: PI }.tiles();
        let q = t[1].apply([1.0, 2.0, 3.0]);
        assert!((q[0] - 1.0).abs() < 1e-5 && (q[1] + 2.0).abs() < 1e-5 && (q[2] - 3.0).abs() < 1e-5, "{q:?}");
    }

    /// A box of `n` cells a side with a linear density `rho = 1 + 2 x + 3 y` (nose diameters ~ metres), one solid cell.
    fn box_field(n: usize) -> Field3 {
        let mut nodes = Vec::new();
        for k in 0..=n {
            for j in 0..=n {
                for i in 0..=n {
                    nodes.push([i as f32 / n as f32, j as f32 / n as f32, k as f32 / n as f32]);
                }
            }
        }
        let mut f = Field3 {
            mach: 2.0,
            alpha_deg: 0.0,
            iteration: 0,
            freestream: [1.0, 2.0, 0.0, 0.0, 1.0 / GAMMA],
            ni: n,
            nj: n,
            nk: n,
            mesh_id: 0,
            nodes,
            solid: vec![false; n * n * n],
            prim: vec![[1.0, 2.0, 0.0, 0.0, 1.0 / GAMMA as f32]; n * n * n],
            eddy_ratio: Vec::new(),
            walls: Vec::new(),
            fin_theta: Vec::new(),
            symmetry: Symmetry::Full,
        };
        for k in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let c = f.centre(i, j, k);
                    let ci = f.cell_index(i, j, k);
                    f.prim[ci][0] = 1.0 + 2.0 * c[0] + 3.0 * c[1];
                }
            }
        }
        f
    }

    #[test]
    fn the_density_gradient_of_a_linear_field_is_its_slope() {
        let f = box_field(6);
        let want = (4.0f32 + 9.0).sqrt();
        for (i, j, k) in [(3, 3, 3), (0, 0, 0), (5, 2, 4)] {
            let g = f.density_gradient_at(i, j, k);
            assert!((g - want).abs() < 1e-3, "({i},{j},{k}) {g} vs {want}");
        }
        // beside a solid cell the difference turns one-sided and stays exact for a linear field
        let mut g = box_field(6);
        let s = g.cell_index(3, 3, 3);
        g.solid[s] = true;
        assert!(g.density_gradient_at(3, 3, 3).is_nan());
        assert!((g.density_gradient_at(2, 3, 3) - want).abs() < 1e-3);
    }

    #[test]
    fn a_one_cell_wedge_has_the_gradient_in_its_plane() {
        // 4 x 4 cells in (x, r), one cell in k, a slight wedge: rho = 1 + 2 x
        let (ni, nj, nk) = (4usize, 4usize, 1usize);
        let mut nodes = Vec::new();
        for k in 0..=nk {
            let th = 0.02 * k as f32;
            for j in 0..=nj {
                for i in 0..=ni {
                    let r = 0.25 * (j as f32 + 1.0);
                    nodes.push([0.25 * i as f32, r * th.sin(), r * th.cos()]);
                }
            }
        }
        let mut f = box_field(2);
        f.ni = ni;
        f.nj = nj;
        f.nk = nk;
        f.nodes = nodes;
        f.solid = vec![false; ni * nj * nk];
        f.prim = vec![[1.0, 2.0, 0.0, 0.0, 0.7]; ni * nj * nk];
        for j in 0..nj {
            for i in 0..ni {
                let c = f.centre(i, j, 0);
                let ci = f.cell_index(i, j, 0);
                f.prim[ci][0] = 1.0 + 2.0 * c[0];
            }
        }
        assert!((f.density_gradient_at(2, 2, 0) - 2.0).abs() < 1e-3, "{}", f.density_gradient_at(2, 2, 0));
    }

    #[test]
    fn mach_number_and_cp_follow_the_free_stream() {
        let mut f = box_field(2);
        let fs = f.freestream;
        f.prim[0] = [fs[0] as f32, fs[1] as f32, 0.0, 0.0, fs[4] as f32];
        assert!((f.mach_number(0) - 2.0).abs() < 1e-5);
        assert!(f.pressure_coefficient(0).abs() < 1e-6);
        f.prim[1][4] = 2.0 * fs[4] as f32;
        assert!((f.pressure_coefficient(1) - (fs[4] as f32 / (0.5 * 4.0))).abs() < 1e-5);
    }

    #[test]
    fn layers_are_found_by_azimuth() {
        let (ni, nj, nk) = (2usize, 2usize, 4usize);
        let mut nodes = Vec::new();
        for k in 0..=nk {
            let th = PI * k as f32 / nk as f32;
            for j in 0..=nj {
                for i in 0..=ni {
                    let r = 1.0 + j as f32;
                    nodes.push([i as f32, r * th.sin(), r * th.cos()]);
                }
            }
        }
        let mut f = box_field(2);
        (f.ni, f.nj, f.nk, f.nodes, f.symmetry) = (ni, nj, nk, nodes, Symmetry::Mirror { theta0: 0.0, theta1: PI });
        assert!((f.theta_of_layer(0) - PI / 8.0).abs() < 1e-5);
        assert_eq!(f.layer_of_theta(0.0), 0);
        assert_eq!(f.layer_of_theta(PI), 3);
        assert_eq!(f.layer_of_theta(PI / 2.0 + 0.01), 2);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn a_field_round_trips_through_json() {
        let mut f = box_field(2);
        f.walls.push(WallFace { nodes: [0, 1, 4, 3], cell: 0, part: Part::Fin(1), cp: 0.5, cf: [0.0, 0.1, 0.0] });
        f.symmetry = Symmetry::Mirror { theta0: 0.0, theta1: PI };
        let back: Field3 = serde_json::from_str(&serde_json::to_string(&f).unwrap()).unwrap();
        assert_eq!(back, f);
    }
}
