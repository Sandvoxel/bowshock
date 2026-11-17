//! Distance of every fluid cell centroid to the nearest no-slip wall of a 3D grid: the 3D counterpart of `turbulence::wall_distance` (which is brute force over
//! wall segments), written for 0.5 M cells against ~15 k wall faces.
//!
//! A wall is any face between a fluid and a solid cell of the per-cell solid mask (`Solver3::solid`, the same classification as [`crate::viscous3`]); faces
//! with `area == 0` are skipped. Nothing here depends on the topology of the grid, on `Grid3::axis_cells` or on where the solids are. The distance to a wall
//! face is the exact distance of the point to the quad of its four nodes split along the diagonal `0-2` into two triangles (closest point of a triangle by
//! the Voronoi-region case analysis of Ericson, *Real-Time Collision Detection* 5.1.5), so it is exact for the warped quads of a curved or revolved grid as
//! far as the quad is its two triangles.
//!
//! ```text
//!   d(c) = min over wall faces f of  dist(xc[c], triangles(f)),     d = +inf in solid cells and when there is no wall face
//! ```
//!
//! The wall triangles go into a bounding-volume hierarchy (median split of the triangle centroids along the longest axis, leaves of up to 4 triangles); a query
//! walks it depth first, nearer child first, pruned by the best distance found, and the cells are queried in parallel. [`wall_distance3_brute`] is the oracle
//! (a plain loop over all triangles) used by the tests. Once per grid (the distance depends on the geometry and the solid mask only).

use crate::grid3::Grid3;
use rayon::prelude::*;

/// One wall face: the family `dir` (0: i, 1: j, 2: k), its index `(i, j, k)` in that family (as [`crate::viscous3::face_of`]) and its four nodes in the cyclic
/// order of `grid3` (right-handed in `(i, j, k)` for the `+index` normal).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallFace {
    pub dir: usize,
    pub idx: [usize; 3],
    pub corners: [[f64; 3]; 4],
}

/// The four nodes of face `(i, j, k)` of the family `dir`, in the order of `Grid3::try_from_nodes`.
pub fn face_corners(grid: &Grid3, dir: usize, i: usize, j: usize, k: usize) -> [[f64; 3]; 4] {
    let (ni, nj) = (grid.ni, grid.nj);
    let pt = |i: usize, j: usize, k: usize| {
        let n = (k * (nj + 1) + j) * (ni + 1) + i;
        [grid.x[n], grid.y[n], grid.z[n]]
    };
    match dir {
        0 => [pt(i, j, k), pt(i, j + 1, k), pt(i, j + 1, k + 1), pt(i, j, k + 1)],
        1 => [pt(i, j, k), pt(i, j, k + 1), pt(i + 1, j, k + 1), pt(i + 1, j, k)],
        _ => [pt(i, j, k), pt(i + 1, j, k), pt(i + 1, j + 1, k), pt(i, j + 1, k)],
    }
}

/// Every no-slip wall face of the grid: the faces between a fluid and a solid cell (non-zero area) and the `j = 0` boundary faces of the runs of
/// [`Grid3::wall_j0`]. A solver with other kinds of fluid/solid faces (a nozzle exit patch) removes them from the list before building the distance field.
pub fn wall_faces(grid: &Grid3, solid: &[bool]) -> Vec<WallFace> {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let mut out = Vec::new();
    for dir in 0..3 {
        let [n0, n1, n2] = [ni + (dir == 0) as usize, nj + (dir == 1) as usize, nk + (dir == 2) as usize];
        for k in 0..n2 {
            for j in 0..n1 {
                for i in 0..n0 {
                    let c = [i, j, k];
                    if c[dir] == 0 || c[dir] == [ni, nj, nk][dir] {
                        continue;
                    }
                    let mut lo = c;
                    lo[dir] -= 1;
                    let (a, b) = (grid.cell(lo[0], lo[1], lo[2]), grid.cell(i, j, k));
                    // a fluid/solid face; the exit of a motor jet (the solid side is a `Part::Jet` cell) is no wall
                    if solid[a] == solid[b] || crate::viscous3::is_jet_cell(grid, if solid[a] { a } else { b }) {
                        continue;
                    }
                    let (_, face) = crate::viscous3::face_of(grid, dir, i, j, k);
                    if face.area == 0.0 {
                        continue;
                    }
                    out.push(WallFace { dir, idx: c, corners: face_corners(grid, dir, i, j, k) });
                }
            }
        }
    }
    for k in 0..nk {
        for (run, _) in &grid.wall_j0 {
            for i in run.clone() {
                if !solid[grid.cell(i, 0, k)] && crate::viscous3::face_of(grid, 1, i, 0, k).1.area > 0.0 {
                    out.push(WallFace { dir: 1, idx: [i, 0, k], corners: face_corners(grid, 1, i, 0, k) });
                }
            }
        }
    }
    out
}

#[inline]
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Squared distance of the point `p` to the triangle `(a, b, c)` (non-degenerate).
pub fn point_triangle_dist2(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return dot(ap, ap); // vertex a
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return dot(bp, bp); // vertex b
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3); // edge ab
        let q = [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]];
        let d = sub(p, q);
        return dot(d, d);
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return dot(cp, cp); // vertex c
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6); // edge ac
        let q = [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]];
        let d = sub(p, q);
        return dot(d, d);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); // edge bc
        let q = [b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2])];
        let d = sub(p, q);
        return dot(d, d);
    }
    let denom = 1.0 / (va + vb + vc); // inside the face
    let (v, w) = (vb * denom, vc * denom);
    let q = [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w];
    let d = sub(p, q);
    dot(d, d)
}

/// The non-degenerate triangles `(0, 1, 2)` and `(0, 2, 3)` of every wall quad.
fn triangles(faces: &[WallFace]) -> Vec<[[f64; 3]; 3]> {
    let mut out = Vec::with_capacity(2 * faces.len());
    for f in faces {
        let q = &f.corners;
        for t in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
            let (u, v) = (sub(t[1], t[0]), sub(t[2], t[0]));
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            if dot(n, n) > 1e-300 {
                out.push(t);
            }
        }
    }
    out
}

/// A node of the hierarchy: the box and either the two children (`count == 0`) or the triangle range `start..start + count`.
#[derive(Clone, Copy)]
struct Node {
    lo: [f64; 3],
    hi: [f64; 3],
    /// Children indices (`count == 0`) or the first triangle.
    a: u32,
    b: u32,
    count: u32,
}

/// Bounding-volume hierarchy over the wall triangles; [`WallBvh::distance`] is the exact distance of a point to the nearest one.
pub struct WallBvh {
    nodes: Vec<Node>,
    tris: Vec<[[f64; 3]; 3]>,
}

const LEAF: usize = 4;

impl WallBvh {
    /// Hierarchy of the triangles of `faces` (empty hierarchy: every distance is infinite).
    pub fn new(faces: &[WallFace]) -> WallBvh {
        let mut tris = triangles(faces);
        let mut nodes = Vec::new();
        if !tris.is_empty() {
            let n = tris.len();
            Self::build(&mut nodes, &mut tris, 0, n);
        }
        WallBvh { nodes, tris }
    }

    fn build(nodes: &mut Vec<Node>, tris: &mut [[[f64; 3]; 3]], start: usize, end: usize) -> u32 {
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let (mut clo, mut chi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for t in &tris[start..end] {
            let cen = [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0, (t[0][2] + t[1][2] + t[2][2]) / 3.0];
            for a in 0..3 {
                for v in t {
                    lo[a] = lo[a].min(v[a]);
                    hi[a] = hi[a].max(v[a]);
                }
                clo[a] = clo[a].min(cen[a]);
                chi[a] = chi[a].max(cen[a]);
            }
        }
        let id = nodes.len() as u32;
        nodes.push(Node { lo, hi, a: start as u32, b: 0, count: (end - start) as u32 });
        if end - start <= LEAF {
            return id;
        }
        let axis = (0..3).max_by(|&x, &y| (chi[x] - clo[x]).total_cmp(&(chi[y] - clo[y]))).unwrap();
        let mid = (start + end) / 2;
        let key = |t: &[[f64; 3]; 3]| t[0][axis] + t[1][axis] + t[2][axis];
        tris[start..end].select_nth_unstable_by(mid - start, |p, q| key(p).total_cmp(&key(q)));
        let l = Self::build(nodes, tris, start, mid);
        let r = Self::build(nodes, tris, mid, end);
        let node = &mut nodes[id as usize];
        node.a = l;
        node.b = r;
        node.count = 0;
        id
    }

    #[inline]
    fn box_dist2(n: &Node, p: [f64; 3]) -> f64 {
        let mut d = 0.0;
        for a in 0..3 {
            let e = (n.lo[a] - p[a]).max(p[a] - n.hi[a]).max(0.0);
            d += e * e;
        }
        d
    }

    /// Distance of `p` to the nearest wall triangle (`f64::INFINITY` without a wall).
    pub fn distance(&self, p: [f64; 3]) -> f64 {
        if self.nodes.is_empty() {
            return f64::INFINITY;
        }
        let mut best = f64::INFINITY;
        let mut stack = [0u32; 96];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let n = &self.nodes[stack[sp] as usize];
            if Self::box_dist2(n, p) >= best {
                continue;
            }
            if n.count > 0 {
                for t in &self.tris[n.a as usize..(n.a + n.count) as usize] {
                    best = best.min(point_triangle_dist2(p, t[0], t[1], t[2]));
                }
            } else {
                let (dl, dr) = (Self::box_dist2(&self.nodes[n.a as usize], p), Self::box_dist2(&self.nodes[n.b as usize], p));
                let (near, far, dn, df) = if dl <= dr { (n.a, n.b, dl, dr) } else { (n.b, n.a, dr, dl) };
                if df < best {
                    stack[sp] = far;
                    sp += 1;
                }
                if dn < best {
                    stack[sp] = near;
                    sp += 1;
                }
            }
        }
        best.sqrt()
    }
}

/// Distance of every fluid cell centroid (`Grid3::xc`) to the nearest of the wall `faces` ([`wall_faces`]); `f64::INFINITY` in solid cells and when there is no
/// wall face. Parallel over the cells, hierarchy of the wall triangles built once.
pub fn wall_distance3(grid: &Grid3, solid: &[bool], faces: &[WallFace]) -> Vec<f64> {
    let bvh = WallBvh::new(faces);
    let mut d = vec![f64::INFINITY; grid.ni * grid.nj * grid.nk];
    d.par_iter_mut().enumerate().for_each(|(c, out)| {
        if !solid[c] {
            *out = bvh.distance(grid.xc[c]);
        }
    });
    d
}

/// The oracle of [`wall_distance3`]: the same distance by a loop over all wall triangles for every cell.
pub fn wall_distance3_brute(grid: &Grid3, solid: &[bool], faces: &[WallFace]) -> Vec<f64> {
    let tris = triangles(faces);
    let mut d = vec![f64::INFINITY; grid.ni * grid.nj * grid.nk];
    d.par_iter_mut().enumerate().for_each(|(c, out)| {
        if !solid[c] {
            let p = grid.xc[c];
            *out = tris.iter().map(|t| point_triangle_dist2(p, t[0], t[1], t[2])).fold(f64::INFINITY, f64::min).sqrt();
        }
    });
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux3::tests::Rng;
    use crate::grid::{cone_grid, ConeGridParams};
    use crate::grid3::{revolve, SolidBox};
    use crate::viscous3::tests::{box_grid, set_solids, solid_mask};

    /// Distance of `p` to the triangle by dense sampling of its barycentric coordinates (upper bound, tight to the sampling step).
    fn sampled_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
        let n = 400;
        let mut best = f64::INFINITY;
        for i in 0..=n {
            for j in 0..=n - i {
                let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                let w = 1.0 - u - v;
                let q = [w * a[0] + u * b[0] + v * c[0], w * a[1] + u * b[1] + v * c[1], w * a[2] + u * b[2] + v * c[2]];
                let d = sub(p, q);
                best = best.min(dot(d, d));
            }
        }
        best.sqrt()
    }

    /// The point-triangle distance against dense sampling for points in every Voronoi region (face, three edges, three vertices) of random triangles.
    #[test]
    fn point_triangle_distance_matches_sampling() {
        let mut rng = Rng::new(3);
        for _ in 0..60 {
            let v = |r: &mut Rng| [r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0)];
            let (a, b, c) = (v(&mut rng), v(&mut rng), v(&mut rng));
            let (u, w) = (sub(b, a), sub(c, a));
            if dot(u, u) * dot(w, w) - dot(u, w) * dot(u, w) < 1e-2 {
                continue;
            }
            for _ in 0..30 {
                let p = [rng.range(-2.0, 2.0), rng.range(-2.0, 2.0), rng.range(-2.0, 2.0)];
                let (exact, sampled) = (point_triangle_dist2(p, a, b, c).sqrt(), sampled_dist(p, a, b, c));
                assert!(exact <= sampled + 1e-12 && sampled - exact < 6e-3, "exact {exact} sampled {sampled}");
            }
        }
    }

    /// A unit square at `z = 0`: the distance is `|z|` above it, `hypot(dx, z)` beside an edge and `hypot(dx, dy, z)` beyond a corner.
    #[test]
    fn quad_distance_in_the_three_regions() {
        let quad = WallFace { dir: 2, idx: [0, 0, 0], corners: [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]] };
        let bvh = WallBvh::new(&[quad]);
        for (p, want) in [
            ([0.3, 0.6, 0.25], 0.25),
            ([0.5, 0.5, -0.7], 0.7),
            ([1.5, 0.5, 0.0], 0.5),
            ([-0.3, 0.2, 0.4], 0.5),
            ([1.3, 1.4, 1.2], (0.09f64 + 0.16 + 1.44).sqrt()),
            ([0.0, 0.0, 0.0], 0.0),
        ] {
            assert!((bvh.distance(p) - want).abs() < 1e-14, "{p:?}: {} vs {want}", bvh.distance(p));
        }
        assert_eq!(WallBvh::new(&[]).distance([0.0; 3]), f64::INFINITY);
    }

    /// A flat wall (a solid row of cells) on a box grid: `d = y_c - y_wall`; the solid cells and a grid without a wall are infinite.
    #[test]
    fn flat_wall_distance_is_the_height() {
        let (n, h) = ([5, 8, 3], [0.1, 0.05, 0.2]);
        let mut g = box_grid(n, h);
        let solid0 = solid_mask(&g);
        assert!(wall_distance3(&g, &solid0, &wall_faces(&g, &solid0)).iter().all(|d| d.is_infinite()), "no wall: infinite everywhere");
        set_solids(&mut g, vec![SolidBox { i: 0..n[0], j: 0..2, k: 0..n[2] }]);
        let solid = solid_mask(&g);
        let faces = wall_faces(&g, &solid);
        assert_eq!(faces.len(), n[0] * n[2], "one wall face per column");
        let d = wall_distance3(&g, &solid, &faces);
        for c in 0..d.len() {
            let want = if solid[c] { f64::INFINITY } else { g.xc[c][1] - 2.0 * h[1] };
            assert!((d[c] - want).abs() < 1e-14 || d[c] == want, "cell {c}: {} vs {want}", d[c]);
        }
    }

    /// The hierarchy against the brute-force oracle, exactly, on a jittered grid with several solid boxes (a thick one, a one-cell fin touching the domain
    /// boundary, a one-cell-thick plate in the middle).
    #[test]
    fn bvh_matches_brute_force_on_a_jittered_grid_with_solids() {
        let mut rng = Rng::new(17);
        let n = [14, 11, 9];
        let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    x.push(0.1 * i as f64 + 0.03 * rng.range(-1.0, 1.0));
                    y.push(0.1 * j as f64 + 0.03 * rng.range(-1.0, 1.0));
                    z.push(0.1 * k as f64 + 0.03 * rng.range(-1.0, 1.0));
                }
            }
        }
        let mut g = Grid3::try_from_nodes(n[0], n[1], n[2], x, y, z).unwrap();
        set_solids(&mut g, vec![SolidBox { i: 2..6, j: 2..5, k: 1..4 }, SolidBox { i: 8..12, j: 0..8, k: 4..5 }, SolidBox { i: 0..3, j: 7..8, k: 6..9 }]);
        let solid = solid_mask(&g);
        let faces = wall_faces(&g, &solid);
        assert!(faces.len() > 100);
        let (a, b) = (wall_distance3(&g, &solid, &faces), wall_distance3_brute(&g, &solid, &faces));
        for c in 0..a.len() {
            assert!(a[c] == b[c] || (a[c] - b[c]).abs() < 1e-14, "cell {c}: {} vs {}", a[c], b[c]);
            assert_eq!(solid[c], a[c].is_infinite());
        }
    }

    /// The same on a revolved grid (degenerate axis faces, warped quads) with a body block: the distance of the cells just above the body surface is about
    /// half the cell height and no distance is zero or negative.
    #[test]
    fn bvh_matches_brute_force_on_a_revolved_grid() {
        let g2 = cone_grid(&ConeGridParams { ni: 24, nj: 12, theta: 12f64.to_radians(), length: 1.0, x_up: -0.4, h_start: 0.8, h_end: 0.8, cluster: 1.5 });
        let nk = 5;
        let theta: Vec<f64> = (0..=nk).map(|k| 0.7 * k as f64 / nk as f64).collect();
        let mut g = revolve(&g2, nk, &theta).unwrap();
        set_solids(&mut g, vec![SolidBox { i: 8..24, j: 0..4, k: 0..nk }]);
        let solid = solid_mask(&g);
        let faces = wall_faces(&g, &solid);
        let (a, b) = (wall_distance3(&g, &solid, &faces), wall_distance3_brute(&g, &solid, &faces));
        for c in 0..a.len() {
            assert!(a[c] == b[c] || (a[c] - b[c]).abs() < 1e-14, "cell {c}: {} vs {}", a[c], b[c]);
            assert!(solid[c] || a[c] > 0.0);
        }
    }

    /// Timing of the 0.5 M cell case (run with `--ignored --no-capture`): a `100 x 100 x 50` grid with two solid blocks whose surface is ~14 k faces.
    #[test]
    #[ignore = "timing"]
    fn timing_of_half_a_million_cells() {
        let n = [100, 100, 50];
        let mut g = box_grid(n, [0.02, 0.02, 0.04]);
        set_solids(&mut g, vec![SolidBox { i: 20..80, j: 20..80, k: 0..24 }, SolidBox { i: 10..14, j: 10..90, k: 24..50 }]);
        let solid = solid_mask(&g);
        let t0 = std::time::Instant::now();
        let faces = wall_faces(&g, &solid);
        let t1 = std::time::Instant::now();
        let d = wall_distance3(&g, &solid, &faces);
        let t2 = std::time::Instant::now();
        let pool = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        let t3 = std::time::Instant::now();
        let d1 = pool.install(|| wall_distance3(&g, &solid, &faces));
        let t4 = std::time::Instant::now();
        assert_eq!(d, d1);
        println!(
            "{} cells, {} wall faces: wall_faces {:?}, wall_distance3 {:?} on {} threads, {:?} on 1 thread",
            d.len(),
            faces.len(),
            t1 - t0,
            t2 - t1,
            rayon::current_num_threads(),
            t4 - t3
        );
    }
}
