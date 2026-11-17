//! 3D structured finite-volume grid: node coordinates, face area vectors, cell volumes.
//!
//! Cell `(i, j, k)` is the hexahedron between node lines `i..=i+1`, `j..=j+1`, `k..=k+1`. The index layouts are fixed by the
//! field docs; faces are oriented towards increasing index regardless of the handedness of the node mapping.

use std::ops::Range;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Face3 {
    /// Unit normal towards the +index direction (`[0; 3]` for a degenerate face).
    pub n: [f64; 3],
    pub area: f64,
    /// Face centroid (mean of its 4 nodes).
    pub c: [f64; 3],
}

/// Block of cells that are not fluid (the body, a sting, fins, cells outside the domain): never updated. Written by the mesher through
/// [`Grid3::mark_solid`], which also fills [`Grid3::part`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SolidBox {
    pub i: Range<usize>,
    pub j: Range<usize>,
    pub k: Range<usize>,
}

/// [`Grid3::part`] of a fluid cell.
pub const PART_FLUID: u8 = 0;
/// [`Grid3::part`] of a cell that is not part of the domain at all (the cells of a body block upstream of the nose tip, whose nodes lie on or
/// behind the axis): never updated, and unlike a wall it does not reflect: the stencils of the fluid cells next to it read the axis ghost.
pub const PART_DEAD: u8 = u8::MAX;

/// What a solid cell belongs to. The mesher writes it, the force integration classifies wall faces from it and the direction of the face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    /// Body segment `n` (`0` the nose, `1..` the aft segments in order); its `+i` face behind the last segment is the base.
    Body(u16),
    /// The sting behind the base.
    Sting,
    /// Fin set `n`.
    Fin(u16),
    /// Appendage `n` (lug, rail button, ...).
    Appendage(u16),
}

/// Equivalent sand-grain roughness height `ks` of the walls by kind of part, in nose diameters (0: smooth). The wall treatment reads it per cell through
/// [`Grid3::wall_ks`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PartRoughness {
    /// The body (every segment) and the sting.
    pub body: f64,
    /// The fins.
    pub fin: f64,
}

impl PartRoughness {
    /// `ks` of the wall of `part`.
    pub fn of(&self, part: Part) -> f64 {
        match part {
            Part::Body(_) | Part::Sting | Part::Appendage(_) => self.body,
            Part::Fin(_) => self.fin,
        }
    }
}

/// How the solved domain relates to the whole body: what the axis ghost states and the force scaling assume.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Symmetry {
    /// `theta in [0, pi]`, mirror planes `theta = 0` and `pi` (the pitch plane): the whole body is the domain and its mirror image.
    Half,
    /// `theta in [0, pi/n]` of a body with `n` identical fins: mirror planes `theta = 0` (a fin mid-plane) and `pi/n` (mid-gap), the rest is
    /// their dihedral group `D_n`.
    Sector { n: u32 },
    /// A single-cell wedge of any angle about an axisymmetric flow (`nk = 1`): the flow is axisymmetric, there is nothing in the `k`
    /// direction (no line solve, no time-step term).
    Wedge,
}

/// Axis cells: the row `j` of cells for `i` in `i` lie on the axis (degenerate faces on their `j` side); the ghost rows are the rows
/// below `j`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AxisRun {
    pub j: usize,
    pub i: Range<usize>,
}

#[derive(Clone, Debug)]
pub struct Grid3 {
    pub ni: usize,
    pub nj: usize,
    pub nk: usize,
    /// Nodes, index `(k*(nj+1)+j)*(ni+1)+i`.
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub z: Vec<f64>,
    /// `(ni+1)*nj*nk`, index `(k*nj+j)*(ni+1)+i`, normal towards +i.
    pub iface: Vec<Face3>,
    /// `ni*(nj+1)*nk`, index `(k*(nj+1)+j)*ni+i`, normal towards +j.
    pub jface: Vec<Face3>,
    /// `ni*nj*(nk+1)`, index `(k*nj+j)*ni+i`, normal towards +k.
    pub kface: Vec<Face3>,
    /// `ni*nj*nk`, index `(k*nj+j)*ni+i`.
    pub vol: Vec<f64>,
    pub xc: Vec<[f64; 3]>,
    /// The boxes of every non-fluid region, as [`Grid3::mark_solid`] was given them (what the GPU solver reads; [`Grid3::part`] is the
    /// per-cell truth).
    pub solid: Vec<SolidBox>,
    /// Per cell: [`PART_FLUID`], [`PART_DEAD`] or `1 + index` into [`Grid3::parts`]. A cell with another value than [`PART_FLUID`] is never
    /// updated; one with a part is a wall for its fluid neighbours.
    pub part: Vec<u8>,
    pub parts: Vec<Part>,
    /// Axis rows (degenerate faces). The `j = 0` boundary outside these runs is a slip wall.
    pub axis: Vec<AxisRun>,
    /// The `j = 0` boundary walls (slip, no solid cells behind them: the body of a sting grid) by `i` range, for the force classes; boundary faces
    /// outside these runs and the axis runs are not counted.
    pub wall_j0: Vec<(Range<usize>, Part)>,
    pub sym: Symmetry,
    /// Azimuth span `theta_max - theta_min` of the domain; the forces of the domain are scaled by `2 pi / theta_span` to the whole body.
    pub theta_span: f64,
    /// Roughness of the walls by part (set by the mesher).
    pub roughness: PartRoughness,
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

impl Grid3 {
    /// Build the grid from node coordinates (`(ni+1)*(nj+1)*(nk+1)`, `i` fastest); `Err` on a negative cell volume.
    /// Zero-volume cells are allowed (fin solid cells). `solid` is empty and `axis_cells` 0; the mesher sets them.
    pub fn try_from_nodes(ni: usize, nj: usize, nk: usize, x: Vec<f64>, y: Vec<f64>, z: Vec<f64>) -> Result<Grid3, String> {
        let nn = (ni + 1) * (nj + 1) * (nk + 1);
        assert_eq!(x.len(), nn);
        assert_eq!(y.len(), nn);
        assert_eq!(z.len(), nn);
        let node = |i: usize, j: usize, k: usize| (k * (nj + 1) + j) * (ni + 1) + i;
        let pt = |i: usize, j: usize, k: usize| {
            let n = node(i, j, k);
            [x[n], y[n], z[n]]
        };
        // Face from its 4 corner nodes in a fixed cyclic order (right-handed in (i, j, k) for the +axis normal); one global
        // handedness sign is applied afterwards, so a folded cell shows up as a negative volume.
        let face = |c: [(usize, usize, usize); 4]| {
            let p = c.map(|(i, j, k)| pt(i, j, k));
            let mut s = cross(sub(p[2], p[0]), sub(p[3], p[1]));
            s = [0.5 * s[0], 0.5 * s[1], 0.5 * s[2]];
            let area = dot(s, s).sqrt();
            let n = if area < 1e-300 { [0.0; 3] } else { [s[0] / area, s[1] / area, s[2] / area] };
            let cen = [0, 1, 2].map(|m| 0.25 * (p[0][m] + p[1][m] + p[2][m] + p[3][m]));
            (Face3 { n, area: if area < 1e-300 { 0.0 } else { area }, c: cen }, s)
        };

        let mut iface = vec![Face3::default(); (ni + 1) * nj * nk];
        let mut si = vec![[0.0; 3]; iface.len()];
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..=ni {
                    let (f, s) = face([(i, j, k), (i, j + 1, k), (i, j + 1, k + 1), (i, j, k + 1)]);
                    let q = (k * nj + j) * (ni + 1) + i;
                    iface[q] = f;
                    si[q] = s;
                }
            }
        }
        let mut jface = vec![Face3::default(); ni * (nj + 1) * nk];
        let mut sj = vec![[0.0; 3]; jface.len()];
        for k in 0..nk {
            for j in 0..=nj {
                for i in 0..ni {
                    let (f, s) = face([(i, j, k), (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j, k)]);
                    let q = (k * (nj + 1) + j) * ni + i;
                    jface[q] = f;
                    sj[q] = s;
                }
            }
        }
        let mut kface = vec![Face3::default(); ni * nj * (nk + 1)];
        let mut sk = vec![[0.0; 3]; kface.len()];
        for k in 0..=nk {
            for j in 0..nj {
                for i in 0..ni {
                    let (f, s) = face([(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k)]);
                    let q = (k * nj + j) * ni + i;
                    kface[q] = f;
                    sk[q] = s;
                }
            }
        }

        let mut vol = vec![0.0; ni * nj * nk];
        let mut xc = vec![[0.0; 3]; vol.len()];
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    let qi = |ii: usize| (k * nj + j) * (ni + 1) + ii;
                    let qj = |jj: usize| (k * (nj + 1) + jj) * ni + i;
                    let qk = |kk: usize| (kk * nj + j) * ni + i;
                    let mut v = 0.0;
                    for (f, s, sign) in [
                        (&iface[qi(i + 1)], &si[qi(i + 1)], 1.0),
                        (&iface[qi(i)], &si[qi(i)], -1.0),
                        (&jface[qj(j + 1)], &sj[qj(j + 1)], 1.0),
                        (&jface[qj(j)], &sj[qj(j)], -1.0),
                        (&kface[qk(k + 1)], &sk[qk(k + 1)], 1.0),
                        (&kface[qk(k)], &sk[qk(k)], -1.0),
                    ] {
                        v += sign * dot(f.c, *s);
                    }
                    let c = (k * nj + j) * ni + i;
                    vol[c] = v / 3.0;
                    let mut m = [0.0; 3];
                    for (dk, dj, di) in (0..8).map(|b| (b >> 2 & 1, b >> 1 & 1, b & 1)) {
                        let p = pt(i + di, j + dj, k + dk);
                        for a in 0..3 {
                            m[a] += 0.125 * p[a];
                        }
                    }
                    xc[c] = m;
                }
            }
        }
        // One global handedness sign: a left-handed node mapping flips every face vector and volume once.
        if vol.iter().sum::<f64>() < 0.0 {
            for f in iface.iter_mut().chain(jface.iter_mut()).chain(kface.iter_mut()) {
                f.n = [-f.n[0], -f.n[1], -f.n[2]];
            }
            for v in vol.iter_mut() {
                *v = -*v;
            }
        }
        let vmax = vol.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    let v = vol[(k * nj + j) * ni + i];
                    if v < -1e-12 * vmax || v.is_nan() {
                        let m = xc[(k * nj + j) * ni + i];
                        return Err(format!("negative cell volume at ({i},{j},{k}), centroid ({:.4}, {:.4}, {:.4}), volume {v:.3e}", m[0], m[1], m[2]));
                    }
                }
            }
        }
        Ok(Grid3 {
            ni,
            nj,
            nk,
            x,
            y,
            z,
            iface,
            jface,
            kface,
            vol,
            xc,
            solid: Vec::new(),
            part: vec![PART_FLUID; ni * nj * nk],
            parts: Vec::new(),
            axis: Vec::new(),
            wall_j0: Vec::new(),
            sym: Symmetry::Half,
            theta_span: std::f64::consts::PI,
            roughness: PartRoughness::default(),
        })
    }

    /// Cell index `(k*nj+j)*ni+i`.
    #[inline]
    pub fn cell(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.nj + j) * self.ni + i
    }

    /// Whether cell `(i, j, k)` is not fluid (a wall or a dead cell): never updated.
    #[inline]
    pub fn is_solid(&self, i: usize, j: usize, k: usize) -> bool {
        self.part[self.cell(i, j, k)] != PART_FLUID
    }

    /// Whether cell `(i, j, k)` is a wall for its fluid neighbours (a solid cell that is not dead).
    #[inline]
    pub fn is_wall(&self, i: usize, j: usize, k: usize) -> bool {
        !matches!(self.part[self.cell(i, j, k)], PART_FLUID | PART_DEAD)
    }

    /// The part of cell `(i, j, k)`, `None` for a fluid or a dead cell.
    pub fn part_of(&self, i: usize, j: usize, k: usize) -> Option<Part> {
        match self.part[self.cell(i, j, k)] {
            PART_FLUID | PART_DEAD => None,
            p => Some(self.parts[usize::from(p) - 1]),
        }
    }

    /// Whether the cells `(i, j)` of every `k` lie on an axis run.
    pub fn is_axis(&self, i: usize, j: usize) -> bool {
        self.axis.iter().any(|r| r.j == j && r.i.contains(&i))
    }

    /// Declare the cells of `b` solid: a wall of `part`, or dead cells (`None`). Later boxes overwrite earlier ones.
    pub fn mark_solid(&mut self, b: SolidBox, part: Option<Part>) {
        let id = match part {
            None => PART_DEAD,
            Some(p) => {
                let n = match self.parts.iter().position(|q| *q == p) {
                    Some(n) => n,
                    None => {
                        self.parts.push(p);
                        self.parts.len() - 1
                    }
                };
                u8::try_from(n + 1).expect("fewer than 254 parts")
            }
        };
        for k in b.k.clone() {
            for j in b.j.clone() {
                for i in b.i.clone() {
                    let c = self.cell(i, j, k);
                    self.part[c] = id;
                }
            }
        }
        self.solid.push(b);
    }

    /// `ks` of the wall each fluid cell touches, per cell (`ni*nj*nk`): the largest roughness over the walls of its faces (a wall is a solid cell of a
    /// part, or a `j = 0` boundary wall of [`Grid3::wall_j0`]; the axis rows touch none), 0 for cells that touch no wall and for solid cells.
    pub fn wall_ks(&self) -> Vec<f64> {
        let (ni, nj, nk) = (self.ni, self.nj, self.nk);
        let mut ks = vec![0.0; ni * nj * nk];
        let j0: Vec<f64> = (0..ni).map(|i| self.wall_j0.iter().find(|(r, _)| r.contains(&i)).map_or(0.0, |(_, p)| self.roughness.of(*p))).collect();
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    if self.is_solid(i, j, k) {
                        continue;
                    }
                    let mut v = if j == 0 && !self.is_axis(i, 0) { j0[i] } else { 0.0 };
                    let near = [
                        (i > 0).then(|| (i - 1, j, k)),
                        (i + 1 < ni).then(|| (i + 1, j, k)),
                        (j > 0).then(|| (i, j - 1, k)),
                        (j + 1 < nj).then(|| (i, j + 1, k)),
                        (k > 0).then(|| (i, j, k - 1)),
                        (k + 1 < nk).then(|| (i, j, k + 1)),
                    ];
                    for (a, b, c) in near.into_iter().flatten() {
                        if let Some(p) = self.part_of(a, b, c) {
                            v = v.max(self.roughness.of(p));
                        }
                    }
                    ks[self.cell(i, j, k)] = v;
                }
            }
        }
        ks
    }
}

/// Radial scale `s_k` of the nodes of the `k` line `theta[k]` that makes the polygonal rings of a revolved grid have the cross-section area of the circular
/// ones: the cell between the lines `k` and `k + 1` is a triangle of area `1/2 r^2 s_k s_{k+1} sin(dtheta)`, the circle's sector `1/2 r^2 dtheta`, so
/// `s_k^2 = w / sin(w)` with `w` the mean width of the two cells adjacent to the line (the one cell at the ends of `theta`; the wedge's cell). Flat-faced
/// cells of the inscribed polygon would otherwise lack the area `1 - sin(dtheta)/dtheta ~ dtheta^2 / 6` and every body would have that much less
/// frontal area and a wake that much narrower.
pub fn polygon_scale(theta: &[f64]) -> Vec<f64> {
    polygon_scale_where(theta, |_| true)
}

/// [`polygon_scale`] with only the cells `k` for which `counted(k)` holds taken into the mean width of a line: the solid layer of a fin (not part of the
/// flow, and wide against the cells beside it) must not set the scale of the nodes at its sides, whose radii would otherwise jump from one line to the next and
/// shear the first cell of the fin's boundary layer. A line none of whose cells counts uses both.
pub fn polygon_scale_where(theta: &[f64], counted: impl Fn(usize) -> bool) -> Vec<f64> {
    let nk = theta.len() - 1;
    let width = |k: usize| theta[k + 1] - theta[k];
    (0..=nk)
        .map(|k| {
            let below = (k > 0).then(|| k - 1);
            let above = (k < nk).then_some(k);
            let cells: Vec<usize> = [below, above].into_iter().flatten().collect();
            let kept: Vec<usize> = cells.iter().copied().filter(|&c| counted(c)).collect();
            let used = if kept.is_empty() { &cells } else { &kept };
            let w = used.iter().map(|&c| width(c)).sum::<f64>() / used.len() as f64;
            (w / w.sin()).sqrt()
        })
        .collect()
}

/// Revolve a 2D grid about the x axis: node `(i, j, k)` is `(x, s_k r sin(theta_k), s_k r cos(theta_k))` with the area-preserving scale `s_k` of
/// [`polygon_scale`], `r` clamped at 0 (the rows of a body block below the axis of a 2D grid have negative radius; their cells collapse onto the axis
/// and are never updated). `theta` has `nk + 1` increasing angles in `(0, pi)` of span (the sectors of this module start at 0; 0 is the +z side, `pi`
/// the -z side).
pub fn revolve(g2: &crate::grid::Grid, nk: usize, theta: &[f64]) -> Result<Grid3, String> {
    if theta.len() != nk + 1 {
        return Err(format!("theta has {} entries, expected {}", theta.len(), nk + 1));
    }
    let n2 = (g2.ni + 1) * (g2.nj + 1);
    let mut x = Vec::with_capacity(n2 * (nk + 1));
    let mut y = Vec::with_capacity(x.capacity());
    let mut z = Vec::with_capacity(x.capacity());
    for (&th, scale) in theta.iter().zip(polygon_scale(theta)) {
        let (s, c) = th.sin_cos();
        for n in 0..n2 {
            let r = g2.r[n].max(0.0) * scale;
            x.push(g2.x[n]);
            y.push(r * s);
            z.push(r * c);
        }
    }
    Grid3::try_from_nodes(g2.ni, g2.nj, nk, x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{cone_grid, ConeGridParams};

    fn closure_check(g: &Grid3) {
        let amax = g.iface.iter().chain(&g.jface).chain(&g.kface).fold(0.0f64, |m, f| m.max(f.area));
        for k in 0..g.nk {
            for j in 0..g.nj {
                for i in 0..g.ni {
                    let mut s = [0.0; 3];
                    let mut add = |f: &Face3, sign: f64| {
                        for m in 0..3 {
                            s[m] += sign * f.area * f.n[m];
                        }
                    };
                    add(&g.iface[(k * g.nj + j) * (g.ni + 1) + i + 1], 1.0);
                    add(&g.iface[(k * g.nj + j) * (g.ni + 1) + i], -1.0);
                    add(&g.jface[(k * (g.nj + 1) + j + 1) * g.ni + i], 1.0);
                    add(&g.jface[(k * (g.nj + 1) + j) * g.ni + i], -1.0);
                    add(&g.kface[((k + 1) * g.nj + j) * g.ni + i], 1.0);
                    add(&g.kface[(k * g.nj + j) * g.ni + i], -1.0);
                    for c in s {
                        assert!(c.abs() <= 1e-13 * amax, "closure {s:?} at ({i},{j},{k})");
                    }
                }
            }
        }
    }

    fn cone() -> crate::grid::Grid {
        cone_grid(&ConeGridParams { ni: 16, nj: 8, theta: 15f64.to_radians(), length: 1.0, x_up: -0.5, h_start: 1.0, h_end: 1.0, cluster: 1.5 })
    }

    #[test]
    fn unit_cube() {
        let n = 3;
        let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
        for k in 0..=n {
            for j in 0..=n {
                for i in 0..=n {
                    x.push(i as f64 / n as f64);
                    y.push(j as f64 / n as f64);
                    z.push(k as f64 / n as f64);
                }
            }
        }
        let g = Grid3::try_from_nodes(n, n, n, x, y, z).unwrap();
        let tot: f64 = g.vol.iter().sum();
        assert!((tot - 1.0).abs() < 1e-14);
        assert!((g.vol[g.cell(1, 1, 1)] - 1.0 / 27.0).abs() < 1e-15);
        assert_eq!(g.iface[0].n, [1.0, 0.0, 0.0]);
        assert_eq!(g.jface[0].n, [0.0, 1.0, 0.0]);
        assert_eq!(g.kface[0].n, [0.0, 0.0, 1.0]);
        assert!((g.iface[0].area - 1.0 / 9.0).abs() < 1e-15);
        assert!(g.solid.is_empty() && g.axis.is_empty() && !g.is_solid(0, 0, 0));
        closure_check(&g);
    }

    #[test]
    fn revolved_cone_volume_and_closure() {
        let g2 = cone();
        let nk = 64;
        let theta: Vec<f64> = (0..=nk).map(|k| std::f64::consts::PI * k as f64 / nk as f64).collect();
        let g = revolve(&g2, nk, &theta).unwrap();
        closure_check(&g);
        // area-preserving rings: each sector of angle d holds d per unit of the 2D per-radian volume, the circle's
        let exact = std::f64::consts::PI * g2.vol.iter().sum::<f64>();
        let got: f64 = g.vol.iter().sum();
        assert!((got - exact).abs() < 1e-10 * exact, "{got} vs {exact}");
    }

    #[test]
    fn polygon_scale_keeps_the_area_of_the_ring() {
        // uniform cells: exact; non-uniform (a thin cell between wide ones, as at a fin): the mean-width scale is second order in the neighbours' difference
        let uniform: Vec<f64> = (0..=8).map(|k| 0.4 * k as f64 / 8.0).collect();
        let uneven = [0.0, 0.05, 0.07, 0.30, 0.55, 0.60, 0.95, 1.30, 1.45];
        for (theta, tol) in [(&uniform[..], 1e-14), (&uneven[..], 5e-3)] {
            let s = polygon_scale(theta);
            let area: f64 = theta.windows(2).enumerate().map(|(k, t)| s[k] * s[k + 1] * (t[1] - t[0]).sin()).sum();
            let circle = theta[theta.len() - 1] - theta[0];
            assert!((area / circle - 1.0).abs() < tol, "ring area {area} vs {circle}");
        }
        // a wedge: the one cell
        let s = polygon_scale(&[0.0, 0.21]);
        assert!((s[0] * s[1] * 0.21f64.sin() - 0.21).abs() < 1e-15);
    }

    #[test]
    fn perturbed_closure() {
        let g2 = cone();
        let theta = [0.0, 0.3, 0.9, 1.2, 2.0, std::f64::consts::PI];
        let nk = 5;
        let mut x = vec![];
        let mut y = vec![];
        let mut z = vec![];
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.01
        };
        for &th in &theta {
            for n in 0..(g2.ni + 1) * (g2.nj + 1) {
                let r = g2.r[n];
                x.push(g2.x[n] + if r > 0.0 { rnd() } else { 0.0 });
                y.push(r * th.sin() + if r > 0.0 { rnd() } else { 0.0 });
                z.push(r * th.cos() + if r > 0.0 { rnd() } else { 0.0 });
            }
        }
        let g = Grid3::try_from_nodes(g2.ni, g2.nj, nk, x, y, z).unwrap();
        closure_check(&g);
    }

    #[test]
    fn folded_cell_is_rejected() {
        let g2 = cone();
        let nk = 4;
        let theta: Vec<f64> = (0..=nk).map(|k| std::f64::consts::PI * k as f64 / nk as f64).collect();
        let (ni, nj) = (g2.ni, g2.nj);
        let mut x = vec![];
        let mut y = vec![];
        let mut z = vec![];
        for &th in &theta {
            for n in 0..(ni + 1) * (nj + 1) {
                x.push(g2.x[n]);
                y.push(g2.r[n] * th.sin());
                z.push(g2.r[n] * th.cos());
            }
        }
        // move interior node (8, 4, 2) past its +i neighbour
        let n = (2 * (nj + 1) + 4) * (ni + 1) + 8;
        x[n] = x[n + 1] + 20.0 * (x[n + 1] - x[n]);
        let e = Grid3::try_from_nodes(ni, nj, nk, x, y, z).unwrap_err();
        assert!(e.starts_with("negative cell volume"), "{e}");
    }
}
