//! One cell layer of a [`Field3`] as the meridian-plane picture: the quad mesh `(x, r)` of the layer with the state of its cells, the
//! shape the side-on picture is built from ([`super::Prepared`]).
//!
//! A layer `k` of the 3D grid is the slab between node layers `k` and `k + 1`; its meridian mesh is the mean of the two, with `r` the
//! distance from the axis. The cell state is taken in the meridian frame: `u` along the axis, `v` along the radius through the cell,
//! `w` around the axis (zero for an axisymmetric flow). The wall Cp is that of the body's wall faces next to the layer's cells.

use bowshock_core::{Field3, Part};

/// A meridian-plane picture of one layer of a 3D flow (see the [module documentation](self)).
#[derive(Clone, Debug, PartialEq)]
pub struct Meridian {
    /// Free-stream Mach number.
    pub mach: f64,
    /// Solver iterations done.
    pub iteration: usize,
    /// Cells along `i` (downstream).
    pub ni: usize,
    /// Cells along `j` (away from the axis).
    pub nj: usize,
    /// Axial node coordinates, `(ni + 1)(nj + 1)`.
    pub x: Vec<f32>,
    /// Radial node coordinates, `(ni + 1)(nj + 1)`.
    pub r: Vec<f32>,
    /// Per cell: not part of the flow.
    pub solid: Vec<bool>,
    /// Per cell density, axial, radial and azimuthal velocity, pressure (the solver's units).
    pub rho: Vec<f32>,
    /// See `rho`.
    pub u: Vec<f32>,
    /// See `rho`.
    pub v: Vec<f32>,
    /// See `rho`.
    pub w: Vec<f32>,
    /// See `rho`.
    pub p: Vec<f32>,
    /// Free-stream density, pressure and dynamic pressure `rho |V|^2 / 2`.
    pub rho_inf: f64,
    /// See `rho_inf`.
    pub p_inf: f64,
    /// See `rho_inf`.
    pub q_inf: f64,
    /// Pressure coefficient along the body wall next to the layer: `(x, Cp)`, ascending in `x`.
    pub wall: Vec<(f64, f64)>,
}

impl Meridian {
    /// The meridian picture of cell layer `k` of `f`.
    pub fn of_layer(f: &Field3, k: usize) -> Meridian {
        let (ni, nj) = (f.ni, f.nj);
        let mut x = Vec::with_capacity((ni + 1) * (nj + 1));
        let mut r = Vec::with_capacity(x.capacity());
        for j in 0..=nj {
            for i in 0..=ni {
                let (a, b) = (f.node(i, j, k), f.node(i, j, k + 1));
                x.push(0.5 * (a[0] + b[0]));
                r.push(0.5 * (a[1].hypot(a[2]) + b[1].hypot(b[2])));
            }
        }
        let layer_theta = f.theta_of_layer(k);
        let n = ni * nj;
        let (mut rho, mut u, mut v, mut w, mut p) =
            (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
        let mut solid = Vec::with_capacity(n);
        for j in 0..nj {
            for i in 0..ni {
                let c = f.cell_index(i, j, k);
                let q = f.prim[c];
                let centre = f.centre(i, j, k);
                let theta = if centre[1].hypot(centre[2]) > 1e-9 { centre[1].atan2(centre[2]) } else { layer_theta };
                let (s, co) = theta.sin_cos();
                solid.push(f.solid[c]);
                rho.push(q[0]);
                u.push(q[1]);
                v.push(q[2] * s + q[3] * co);
                w.push(q[3] * s - q[2] * co);
                p.push(q[4]);
            }
        }
        let fs = f.freestream;
        let mut wall: Vec<(f64, f64)> = f
            .walls
            .iter()
            .filter(|wf| matches!(wf.part, Part::Nose | Part::Body | Part::Base | Part::Other) && wf.cell as usize / n == k)
            .map(|wf| (wf.nodes.iter().map(|&q| f64::from(f.nodes[q as usize][0])).sum::<f64>() / 4.0, f64::from(wf.cp)))
            .collect();
        wall.sort_by(|a, b| a.0.total_cmp(&b.0));
        Meridian {
            mach: f.mach,
            iteration: f.iteration,
            ni,
            nj,
            x,
            r,
            solid,
            rho,
            u,
            v,
            w,
            p,
            rho_inf: fs[0],
            p_inf: fs[4],
            q_inf: 0.5 * fs[0] * (fs[1] * fs[1] + fs[2] * fs[2] + fs[3] * fs[3]),
            wall,
        }
    }

    /// Number of cells, `ni * nj`.
    pub fn cells(&self) -> usize {
        self.ni * self.nj
    }

    /// Coordinates `(x, r)` of node `(i, j)`.
    pub fn node(&self, i: usize, j: usize) -> (f32, f32) {
        let k = j * (self.ni + 1) + i;
        (self.x[k], self.r[k])
    }

    /// Centre `(x, r)` of cell `(i, j)`: the mean of its four nodes.
    pub fn centre(&self, i: usize, j: usize) -> (f32, f32) {
        let (a, b, c, d) = (self.node(i, j), self.node(i + 1, j), self.node(i + 1, j + 1), self.node(i, j + 1));
        (0.25 * (a.0 + b.0 + c.0 + d.0), 0.25 * (a.1 + b.1 + c.1 + d.1))
    }

    /// Whether cell `(i, j)` is part of the flow.
    pub fn is_fluid(&self, i: usize, j: usize) -> bool {
        !self.solid[j * self.ni + i]
    }

    /// The first row of column `i` that is part of the flow (`nj` if the whole column is solid).
    pub fn first_fluid_row(&self, i: usize) -> usize {
        (0..self.nj).find(|&j| self.is_fluid(i, j)).unwrap_or(self.nj)
    }

    /// Mach number of cell `c` (`j * ni + i`).
    pub fn mach_number(&self, c: usize) -> f32 {
        (self.u[c] * self.u[c] + self.v[c] * self.v[c] + self.w[c] * self.w[c]).sqrt() / (bowshock_core::field::GAMMA as f32 * self.p[c] / self.rho[c]).sqrt()
    }

    /// Pressure coefficient of cell `c`: `(p - p_inf) / (rho_inf |V_inf|^2 / 2)`.
    pub fn pressure_coefficient(&self, c: usize) -> f32 {
        ((f64::from(self.p[c]) - self.p_inf) / self.q_inf) as f32
    }

    /// Mach number per cell; NaN in solid cells.
    pub fn mach_numbers(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.mach_number(c) }).collect()
    }

    /// Pressure coefficient per cell; NaN in solid cells.
    pub fn pressure_coefficients(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.pressure_coefficient(c) }).collect()
    }

    /// Magnitude of the gradient of the density (over the free-stream density) per cell, per unit of length of the grid (1/m): what a
    /// schlieren picture shows. From differences of the cell-centre values on the curvilinear grid: central along each grid direction,
    /// one-sided at the boundaries and beside solid cells. NaN in solid cells.
    pub fn density_gradient(&self) -> Vec<f32> {
        let (ni, nj) = (self.ni, self.nj);
        let centres: Vec<(f32, f32)> = (0..self.cells()).map(|c| self.centre(c % ni, c / ni)).collect();
        let fluid = |i: usize, j: usize| !self.solid[j * ni + i];
        let rho_inf = self.rho_inf as f32;
        let mut out = vec![f32::NAN; self.cells()];
        for j in 0..nj {
            for i in 0..ni {
                if !fluid(i, j) {
                    continue;
                }
                let c = j * ni + i;
                // the two ends of the difference along i and along j: the neighbours where they are flow cells, else the cell itself
                let a = if i > 0 && fluid(i - 1, j) { c - 1 } else { c };
                let b = if i + 1 < ni && fluid(i + 1, j) { c + 1 } else { c };
                let d = if j > 0 && fluid(i, j - 1) { c - ni } else { c };
                let e = if j + 1 < nj && fluid(i, j + 1) { c + ni } else { c };
                let diff = |hi: usize, lo: usize| (centres[hi].0 - centres[lo].0, centres[hi].1 - centres[lo].1, self.rho[hi] - self.rho[lo]);
                let (x_xi, r_xi, rho_xi) = diff(b, a);
                let (x_eta, r_eta, rho_eta) = diff(e, d);
                let det = x_xi * r_eta - r_xi * x_eta;
                out[c] = if det.abs() > 0.0 {
                    let gx = (rho_xi * r_eta - rho_eta * r_xi) / det;
                    let gr = (rho_eta * x_xi - rho_xi * x_eta) / det;
                    gx.hypot(gr) / rho_inf
                } else {
                    // a single row or column of flow cells: the gradient along the one direction that has a difference
                    let along = |dx: f32, dr: f32, drho: f32| if dx.hypot(dr) > 0.0 { drho.abs() / dx.hypot(dr) } else { 0.0 };
                    along(x_xi, r_xi, rho_xi).max(along(x_eta, r_eta, rho_eta)) / rho_inf
                };
            }
        }
        out
    }

    /// The lower edge of the flow in the meridian plane, from the inflow plane to the outflow plane, as a polyline of `[x, r]`: the
    /// axis ahead of the nose tip, the body surface, the flat base and the sting or the axis behind it. The body is what lies between
    /// this line and the axis.
    pub fn surface(&self) -> Vec<[f32; 2]> {
        let mut line: Vec<[f32; 2]> = Vec::with_capacity(self.ni + 3);
        let mut push = |p: (f32, f32)| {
            if line.last() != Some(&[p.0, p.1]) {
                line.push([p.0, p.1]);
            }
        };
        for i in 0..self.ni {
            let j = self.first_fluid_row(i);
            push(self.node(i, j));
            push(self.node(i + 1, j));
        }
        line
    }
}

#[cfg(test)]
mod tests {
    use super::super::axi::field3_from_2d;
    use super::super::prepared::tests::{sized2d, small2d};
    use super::*;
    use bowshock_core::Field;

    fn assert_close(a: &[f32], b: &[f32], what: &str) {
        assert_eq!(a.len(), b.len(), "{what}");
        for (i, (x, y)) in a.iter().zip(b).enumerate() {
            let same = (x.is_nan() && y.is_nan()) || (x - y).abs() <= 2e-5 * (1.0 + x.abs().max(y.abs()));
            assert!(same, "{what}[{i}]: {x} vs {y}");
        }
    }

    /// The 2D side-on picture fed through the adapter has what the 2D `Field` gives, array for array: the regression that nothing the
    /// current viewer shows is lost when the picture comes from a `Field3` (delete with the 2D `Field`).
    fn same_as_the_2d_field(f: &Field) {
        let f3 = field3_from_2d(f);
        let m = Meridian::of_layer(&f3, 0);
        assert_eq!((m.ni, m.nj, m.mach, m.iteration), (f.ni, f.nj, f.mach, f.iteration));
        assert_close(&m.x, &f.x, "x");
        assert_close(&m.r, &f.r, "r");
        assert_eq!(m.solid, f.solid);
        assert_close(&m.rho, &f.rho, "rho");
        assert_close(&m.u, &f.u, "u");
        assert_close(&m.v, &f.v, "v");
        assert!(m.w.iter().all(|w| w.abs() < 1e-6));
        assert_close(&m.p, &f.p, "p");
        assert_close(&m.mach_numbers(), &f.mach_numbers(), "Mach");
        assert_close(&m.pressure_coefficients(), &f.pressure_coefficients(), "Cp");
        assert_close(&m.density_gradient(), &f.density_gradient(), "density gradient");
        let (a, b) = (m.surface(), f.surface());
        assert_eq!(a.len(), b.len());
        for (p, q) in a.iter().zip(&b) {
            assert!((p[0] - q[0]).abs() < 1e-5 && (p[1] - q[1]).abs() < 1e-5, "{p:?} vs {q:?}");
        }
        let wall = f.wall();
        assert_eq!(m.wall.len(), wall.len());
        for (w, v) in m.wall.iter().zip(&wall) {
            assert!((w.0 - v.x).abs() < 1e-5 && (w.1 - v.cp).abs() < 1e-4, "{w:?} vs {v:?}");
        }
    }

    #[test]
    fn the_adapter_keeps_what_the_2d_field_gives_on_the_small_fixture() {
        same_as_the_2d_field(&small2d(2.0));
        same_as_the_2d_field(&small2d(0.6));
    }

    #[test]
    fn the_adapter_keeps_what_the_2d_field_gives_on_the_app_sized_fixture() {
        same_as_the_2d_field(&sized2d(220, 124));
    }

    #[test]
    fn the_wedge_has_one_layer_two_tiles_per_5_degrees_and_walls_that_face_the_flow() {
        let f3 = field3_from_2d(&small2d(2.0));
        assert_eq!((f3.nk, f3.cells(), f3.symmetry.tiles().len()), (1, 12, 72));
        for w in &f3.walls {
            assert!(f3.is_fluid(w.cell as usize));
            let p = |n: u32| f3.nodes[n as usize];
            let (a, b) = (p(w.nodes[2]), p(w.nodes[0]));
            let (c, d) = (p(w.nodes[3]), p(w.nodes[1]));
            let (d02, d13) = ([a[0] - b[0], a[1] - b[1], a[2] - b[2]], [c[0] - d[0], c[1] - d[1], c[2] - d[2]]);
            let n = [d02[1] * d13[2] - d02[2] * d13[1], d02[2] * d13[0] - d02[0] * d13[2], d02[0] * d13[1] - d02[1] * d13[0]];
            let cell = f3.centre(w.cell as usize % f3.ni, w.cell as usize / f3.ni, 0);
            let face: Vec<f32> = (0..3).map(|m| 0.25 * w.nodes.iter().map(|&q| p(q)[m]).sum::<f32>()).collect();
            let toward = [cell[0] - face[0], cell[1] - face[1], cell[2] - face[2]];
            assert!(n[0] * toward[0] + n[1] * toward[1] + n[2] * toward[2] > 0.0, "{w:?}");
        }
        assert_eq!(f3.mesh_id, bowshock_core::engine::field3::mesh_id(&f3));
    }
}
