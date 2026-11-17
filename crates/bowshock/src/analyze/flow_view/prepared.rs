//! A flow field made ready to draw: the three scalars per cell, their colour ranges, the vertices the GPU draws (one per grid node,
//! the mean of the flow cells around it), the body's outline, the wall Cp, and the lookup of the cell under the pointer.
//!
//! Built once per picture on the UI thread, when the picture changes (a snapshot arrives, another Mach number is chosen): the
//! cost is a few passes over the cells (measured in the module tests of `flow_view`).

use super::map::Scalar;
use super::meridian::Meridian;
use bowshock_core::Field3;
use std::sync::atomic::{AtomicU64, Ordering};
#[cfg(test)]
use std::sync::Arc;

/// One vertex of the picture: a grid node and the three scalars there (Mach, Cp, log10 of the density gradient).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub pos: [f32; 2],
    pub values: [f32; 3],
}

/// Floor of the density gradient (per unit of length) before the logarithm: nothing below it is told from the free stream.
const GRADIENT_FLOOR: f32 = 1e-3;
/// Decades of density gradient the schlieren ramp spans.
pub const SCHLIEREN_DECADES: f32 = 3.0;

/// A field ready to draw.
#[derive(Debug)]
pub struct Prepared {
    /// Changes with every picture: the GPU side uploads a picture's vertices when it sees a new one.
    pub id: u64,
    /// The meridian layer of the field this picture shows.
    pub mer: Meridian,
    /// Mach number per cell (NaN in blocked cells).
    pub mach: Vec<f32>,
    /// Pressure coefficient per cell.
    pub cp: Vec<f32>,
    /// Magnitude of the density gradient over the free-stream density per cell, per unit of length of the field's coordinates.
    pub gradient: Vec<f32>,
    /// The colour range of each [`Scalar`]: Mach from 0, Cp symmetric about 0, the schlieren in log10 over [`SCHLIEREN_DECADES`].
    pub ranges: [(f32, f32); 3],
    /// The upper half of the body in the meridian plane: from the nose tip down the surface to the base, then to the axis.
    pub outline: Vec<[f32; 2]>,
    /// Pressure coefficient along the body surface: `(x, Cp)` in the field's length unit.
    pub wall: Vec<(f64, f64)>,
    /// Length of the body (the axial position of the last surface point off the axis) and its largest radius.
    pub length: f32,
    pub radius: f32,
    /// One per grid node.
    pub vertices: Vec<Vertex>,
    /// Triangles of the flow cells (three indices each), blocked cells left out.
    pub triangles: Vec<u32>,
    /// The edges of the flow cells (two indices each), shared edges once.
    pub edges: Vec<u32>,
}

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

impl Prepared {
    /// Derives everything the viewer draws from the meridian layer of `field` nearest the lee side of the pitch plane (azimuth 0).
    #[cfg(test)]
    pub fn new(field: Arc<Field3>) -> Prepared {
        let layer = field.layer_of_theta(0.0);
        Prepared::of_layer(&field, layer)
    }

    /// Derives everything the viewer draws from cell layer `layer` of `field`.
    pub fn of_layer(field: &Field3, layer: usize) -> Prepared {
        let mer = Meridian::of_layer(field, layer);
        let f = &mer;
        let (ni, nj) = (f.ni, f.nj);
        let mach = f.mach_numbers();
        let cp = f.pressure_coefficients();
        let gradient = f.density_gradient();
        let fluid = |c: usize| !f.solid[c];

        let finite = |v: &[f32]| -> Vec<f32> { v.iter().copied().filter(|x| x.is_finite()).collect() };
        let mach_max = finite(&mach).into_iter().fold(0.0f32, f32::max);
        let cp_max = finite(&cp).into_iter().fold(0.0f32, |m, c| m.max(c.abs()));
        let mut g = finite(&gradient);
        let top = if g.is_empty() {
            1.0
        } else {
            // the strongest gradients (a shock in one cell) set the top; the last half per cent is left to saturate
            let k = ((g.len() as f32 * 0.995) as usize).min(g.len() - 1);
            *g.select_nth_unstable_by(k, f32::total_cmp).1
        };
        let hi = top.max(10.0).log10();
        let ranges = [(0.0, ((mach_max * 4.0).ceil() / 4.0).max(1.0)), (-1.0, 1.0), (hi - SCHLIEREN_DECADES, hi)];
        let cp_range = ((cp_max * 10.0).ceil() / 10.0).max(0.2);
        let ranges = [ranges[0], (-cp_range, cp_range), ranges[2]];

        // the scalars at the nodes: the mean of the flow cells that share the node
        let cell_values = |c: usize| [mach[c], cp[c], gradient[c].max(GRADIENT_FLOOR).log10()];
        let mut vertices = Vec::with_capacity((ni + 1) * (nj + 1));
        for j in 0..=nj {
            for i in 0..=ni {
                let (x, r) = f.node(i, j);
                let mut sum = [0.0f32; 3];
                let mut n = 0u32;
                for (ci, cj) in [(i.wrapping_sub(1), j.wrapping_sub(1)), (i, j.wrapping_sub(1)), (i.wrapping_sub(1), j), (i, j)] {
                    if ci < ni && cj < nj && fluid(cj * ni + ci) {
                        let v = cell_values(cj * ni + ci);
                        sum.iter_mut().zip(v).for_each(|(s, v)| *s += v);
                        n += 1;
                    }
                }
                let values = if n == 0 { [f32::NAN; 3] } else { sum.map(|s| s / n as f32) };
                vertices.push(Vertex { pos: [x, r], values });
            }
        }
        let node = |i: usize, j: usize| (j * (ni + 1) + i) as u32;
        let mut triangles = Vec::new();
        let mut edges = Vec::new();
        for j in 0..nj {
            for i in 0..ni {
                if !fluid(j * ni + i) {
                    continue;
                }
                let (a, b, c, d) = (node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1));
                triangles.extend([a, b, c, a, c, d]);
                // the edges below and to the left of the cell; those above and to the right where no flow cell is there to draw them
                edges.extend([a, b, a, d]);
                if j + 1 == nj || !fluid((j + 1) * ni + i) {
                    edges.extend([d, c]);
                }
                if i + 1 == ni || !fluid(j * ni + i + 1) {
                    edges.extend([b, c]);
                }
            }
        }

        // the body: the surface from the nose tip to its last point off the axis, closed on the axis
        let surface = f.surface();
        let start = surface.iter().position(|p| p[0] >= -1e-9).unwrap_or(0);
        let end = surface.iter().rposition(|p| p[1] > 1e-9).map_or(start, |e| e + 1);
        let mut outline: Vec<[f32; 2]> = surface[start..end.max(start)].to_vec();
        if let Some(last) = outline.last().copied() {
            outline.push([last[0], 0.0]);
        }
        let length = outline.iter().map(|p| p[0]).fold(0.0f32, f32::max);
        let radius = outline.iter().map(|p| p[1]).fold(0.0f32, f32::max);
        let wall = f.wall.clone();

        Prepared { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), mer, mach, cp, gradient, ranges, outline, wall, length, radius, vertices, triangles, edges }
    }

    /// The box the first view shows, `(x0, x1, r_max)`: the body, a little of the flow ahead of it and the wake behind. Falls back
    /// to the whole grid for a field without a body in it.
    pub fn fit_box(&self) -> (f32, f32, f32) {
        if self.length <= 0.0 {
            let f = &self.mer;
            let (x0, x1) = f.x.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
            let r1 = f.r.iter().copied().fold(0.0f32, f32::max);
            return (x0, x1, r1.max(1e-6));
        }
        let l = self.length;
        // the width decides how large the body is; the height only has to hold a few body radii each side of the axis
        (-0.12 * l, 1.45 * l, 2.5 * self.radius)
    }

    /// The flow cell that holds the point `(x, r)` of the meridian plane (`r` is a radius: the picture is mirrored).
    pub fn cell_at(&self, x: f32, r: f32) -> Option<usize> {
        let f = &self.mer;
        let (ni, nj) = (f.ni, f.nj);
        for j in 0..nj {
            for i in 0..ni {
                let c = j * ni + i;
                if f.solid[c] {
                    continue;
                }
                let q = [f.node(i, j), f.node(i + 1, j), f.node(i + 1, j + 1), f.node(i, j + 1)];
                let (lo_x, hi_x) = q.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), p| (a.min(p.0), b.max(p.0)));
                if x < lo_x || x > hi_x {
                    continue;
                }
                let (lo_r, hi_r) = q.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), p| (a.min(p.1), b.max(p.1)));
                if r < lo_r || r > hi_r {
                    continue;
                }
                // inside the convex quad: on the same side of all four edges
                let side = |a: (f32, f32), b: (f32, f32)| (b.0 - a.0) * (r - a.1) - (b.1 - a.1) * (x - a.0);
                let s = [side(q[0], q[1]), side(q[1], q[2]), side(q[2], q[3]), side(q[3], q[0])];
                if s.iter().all(|v| *v >= 0.0) || s.iter().all(|v| *v <= 0.0) {
                    return Some(c);
                }
            }
        }
        None
    }

    /// The wall Cp at `x` by linear interpolation along the wall, `None` off the body.
    pub fn wall_cp_at(&self, x: f64) -> Option<f64> {
        let i = self.wall.partition_point(|p| p.0 < x);
        match (i.checked_sub(1).map(|k| self.wall[k]), self.wall.get(i)) {
            (Some(a), Some(b)) if b.0 > a.0 => Some(a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)),
            _ => None,
        }
    }

    /// The value of `scalar` in cell `c` as the picture colours it (the schlieren as the log10 of the gradient).
    pub fn value(&self, scalar: Scalar, c: usize) -> f32 {
        match scalar {
            Scalar::Mach => self.mach[c],
            Scalar::Cp => self.cp[c],
            Scalar::Schlieren => self.gradient[c].max(GRADIENT_FLOOR).log10(),
        }
    }

    /// Makes this picture and `other` (the other half of the plane) colour by the same ranges: those of both together.
    pub fn share_ranges(&mut self, other: &mut Prepared) {
        let mut ranges = self.ranges;
        for (s, r) in ranges.iter_mut().enumerate() {
            let (a, b) = (self.ranges[s], other.ranges[s]);
            *r = (a.0.min(b.0), a.1.max(b.1));
        }
        // the schlieren ramp is a fixed number of decades below its top
        ranges[Scalar::Schlieren.index()].0 = ranges[Scalar::Schlieren.index()].1 - SCHLIEREN_DECADES;
        self.ranges = ranges;
        other.ranges = ranges;
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::super::axi::field3_from_2d;
    use super::super::map::Scalar;
    use super::*;
    use bowshock_core::{field::GAMMA, Field, Wake};

    /// [`small2d`] as the viewer takes it: the wedge of a `Field3`.
    pub(in crate::analyze) fn small(mach: f64) -> Field3 {
        field3_from_2d(&small2d(mach))
    }

    /// [`sized2d`] as the viewer takes it: the wedge of a `Field3`.
    pub(in crate::analyze) fn sized(ni: usize, nj: usize) -> Field3 {
        field3_from_2d(&sized2d(ni, nj))
    }

    /// A 4 x 3 grid of unit squares: x from -1 (ahead of the nose tip at 0), r from 0, with a 'body' of blocked cells at columns 1 and 2
    /// of row 0 (the body spans x in [0, 2], r in [0, 1]) and a pressure that rises with x.
    pub(in crate::analyze) fn small2d(mach: f64) -> Field {
        let (ni, nj) = (4usize, 3usize);
        let (mut x, mut r) = (Vec::new(), Vec::new());
        for j in 0..=nj {
            for i in 0..=ni {
                x.push(i as f32 - 1.0);
                r.push(j as f32);
            }
        }
        let n = ni * nj;
        let mut solid = vec![false; n];
        solid[1] = true;
        solid[2] = true;
        let p0 = (1.0 / GAMMA) as f32;
        Field {
            mach,
            iteration: 500,
            wake: Wake::Base,
            ni,
            nj,
            x,
            r,
            solid,
            rho: (0..n).map(|c| 1.0 + 0.1 * (c % ni) as f32).collect(),
            u: vec![mach as f32; n],
            v: vec![0.0; n],
            p: (0..n).map(|c| p0 * (1.0 + 0.2 * (c % ni) as f32)).collect(),
            freestream: [1.0, mach, 0.0, 1.0 / GAMMA],
        }
    }

    /// A grid of `ni` x `nj` cells with a flow in it: x from -0.2 to 1.6, r from 0 to 0.3 (finer at the axis), a blocked 'body' of
    /// 60 % of the columns and two rows, the density rising downstream, at Mach 2. 220 x 124 is the size of the app's coarse grid.
    pub(in crate::analyze) fn sized2d(ni: usize, nj: usize) -> Field {
        let (mut x, mut r) = (Vec::new(), Vec::new());
        for j in 0..=nj {
            for i in 0..=ni {
                x.push(-0.2 + 1.8 * i as f32 / ni as f32);
                r.push(0.3 * (j as f32 / nj as f32).powi(2));
            }
        }
        let n = ni * nj;
        let mut solid = vec![false; n];
        for i in ni / 9..ni * 7 / 9 {
            solid[i] = true;
            solid[ni + i] = true;
        }
        let p0 = (1.0 / GAMMA) as f32;
        Field {
            mach: 2.0,
            iteration: 1000,
            wake: Wake::Base,
            ni,
            nj,
            x,
            r,
            solid,
            rho: (0..n).map(|c| 1.0 + 0.5 * ((c % ni) as f32 / ni as f32)).collect(),
            u: vec![2.0; n],
            v: vec![0.0; n],
            p: (0..n).map(|c| p0 * (1.0 + 0.4 * (((c / ni) as f32) / nj as f32))).collect(),
            freestream: [1.0, 2.0, 0.0, 1.0 / GAMMA],
        }
    }

    #[test]
    fn the_vertices_are_the_means_of_the_flow_cells_around_each_node_and_blocked_cells_are_not_drawn() {
        let f = small(2.0);
        let p = Prepared::new(Arc::new(f.clone()));
        assert_eq!(p.vertices.len(), 5 * 4);
        // 12 cells, 2 blocked
        assert_eq!(p.triangles.len(), 10 * 6);
        assert!(p.triangles.iter().all(|&i| (i as usize) < p.vertices.len()));
        // node (0, 0) touches cell (0, 0) only: its Mach number is that cell's
        let v = p.vertices[0];
        assert_eq!(v.pos, [-1.0, 0.0]);
        assert!((v.values[0] - p.mach[0]).abs() < 1e-6);
        // node (2, 1) touches cells (1,0) and (2,0) (blocked), (1,1) and (2,1): the mean of the two flow cells
        let w = p.vertices[5 + 2];
        assert!((w.values[1] - 0.5 * (p.cp[4 + 1] + p.cp[4 + 2])).abs() < 1e-6);
        // a node with no flow cell around it (the body's axis nodes at i = 2) is NaN, and no triangle uses it
        assert!(p.vertices[2].values.iter().all(|x| x.is_nan()));
        assert!(p.triangles.iter().all(|&i| p.vertices[i as usize].values.iter().all(|x| x.is_finite())));
        // every cell has its edges once: 10 flow cells give a bottom and a left edge each, plus the edges on the open sides
        assert!(p.edges.len().is_multiple_of(2) && p.edges.iter().all(|&i| (i as usize) < p.vertices.len()));
    }

    #[test]
    fn the_ranges_start_mach_at_zero_centre_cp_on_zero_and_span_three_decades_of_gradient() {
        let p = Prepared::new(Arc::new(small(2.0)));
        let (m, cp, s) = (p.ranges[Scalar::Mach.index()], p.ranges[Scalar::Cp.index()], p.ranges[Scalar::Schlieren.index()]);
        assert_eq!(m.0, 0.0);
        assert!(m.1 >= 2.0 && m.1 <= 2.25 + 1e-6, "{m:?}");
        assert_eq!(cp.0, -cp.1);
        assert!(cp.1 >= 0.2);
        assert!((s.1 - s.0 - SCHLIEREN_DECADES).abs() < 1e-6);
        // the free stream (no gradient) is below the schlieren range's top
        let mut quiet = small(2.0);
        quiet.prim.iter_mut().for_each(|q| q[0] = 1.0);
        let q = Prepared::new(Arc::new(quiet));
        assert!(q.gradient.iter().all(|g| g.is_nan() || *g < 1e-6));
        assert!(q.ranges[2].1 >= 1.0);
    }

    #[test]
    fn the_body_outline_runs_from_the_tip_to_the_base_and_down_to_the_axis() {
        let p = Prepared::new(Arc::new(small(2.0)));
        assert_eq!(p.outline.first(), Some(&[0.0, 0.0]));
        assert_eq!(p.outline.last(), Some(&[2.0, 0.0]));
        assert!(p.outline.contains(&[0.0, 1.0]) && p.outline.contains(&[2.0, 1.0]));
        assert_eq!((p.length, p.radius), (2.0, 1.0));
        let (x0, x1, r) = p.fit_box();
        assert!(x0 < 0.0 && x1 > p.length && r >= p.radius);
    }

    #[test]
    fn the_pointer_finds_its_cell_and_nothing_in_the_body_or_outside() {
        let p = Prepared::new(Arc::new(small(2.0)));
        // x from -1: column 0 is x in [-1, 0]; row 2 is r in [2, 3]
        assert_eq!(p.cell_at(-0.5, 0.5), Some(0));
        assert_eq!(p.cell_at(2.5, 2.5), Some(2 * 4 + 3));
        assert_eq!(p.cell_at(0.5, 0.5), None, "inside the body");
        assert_eq!(p.cell_at(0.5, 1.5), Some(4 + 1));
        assert_eq!(p.cell_at(5.0, 0.5), None);
        assert_eq!(p.cell_at(0.5, -0.5), None);
    }

    #[test]
    fn the_wall_cp_is_interpolated_along_the_body() {
        let mut p = Prepared::new(Arc::new(small(2.0)));
        p.wall = vec![(0.0, 1.0), (1.0, 0.0), (2.0, -1.0)];
        assert_eq!(p.wall_cp_at(0.5), Some(0.5));
        assert_eq!(p.wall_cp_at(1.5), Some(-0.5));
        assert_eq!(p.wall_cp_at(-0.5), None);
        assert_eq!(p.wall_cp_at(2.5), None);
    }
}
