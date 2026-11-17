//! What is under the pointer in the 3D scene: a ray from the camera against the wall triangles and the cut sheets, on the CPU.
//!
//! The GPU draws the same meshes once per tile (a copy of the solved domain); here the ray is mapped into the solved domain through each
//! tile's inverse instead, so one triangle list serves every copy. Brute force over the triangles that are drawn (the wall faces of the
//! vehicle, at most a few 10^5 triangles, and the sheets in view): about a millisecond in a release build for the finned level 0 grid.

use super::scene3::{Prepared3, SheetDraw, TileSel, Window};
use bowshock_core::Tile;

/// What a ray met.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HitKind {
    /// A cut sheet: the flow cell `(i, j, k)` index (`Field3::cell_index` order) the triangle belongs to.
    Sheet { cell: usize },
    /// A wall face: the index into [`bowshock_core::Field3::walls`].
    Wall { face: usize },
}

/// The nearest thing a ray met.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hit {
    pub kind: HitKind,
    /// Distance along the ray (metres).
    pub t: f32,
    /// Where, in the solved domain's frame (the solver's `(x, y, z)`).
    pub point: [f32; 3],
    /// The tile (copy of the solved domain) the ray entered.
    pub tile: usize,
}

/// The solver-frame point of a world-frame point (the inverse of [`super::scene3::world`]).
fn solver(p: [f32; 3]) -> [f32; 3] {
    [p[2], p[0], p[1]]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Möller-Trumbore, double sided: the distance along `d` from `o` to the triangle, if it is hit ahead of `o`.
fn triangle(o: [f32; 3], d: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> Option<f32> {
    let (e1, e2) = (sub(b, a), sub(c, a));
    let p = cross(d, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub(o, a);
    let u = dot(s, p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(d, q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = dot(e2, q) * inv;
    (t > 0.0).then_some(t)
}

/// The nearest hit of the ray `origin + t dir` (world frame, `dir` a unit vector) among the vehicle's wall triangles (`walls`, in every
/// tile when `tile_walls`, else the solved domain only) and the `sheets` as drawn, the sheets clipped to `window`.
pub fn pick(picture: &Prepared3, origin: [f32; 3], dir: [f32; 3], sheets: &[SheetDraw], window: Window, walls: bool, tile_walls: bool) -> Option<Hit> {
    let (o, d) = (solver(origin), solver(dir));
    let mut best: Option<Hit> = None;
    let mut consider = |hit: Hit| {
        if best.is_none_or(|b| hit.t < b.t) {
            best = Some(hit);
        }
    };
    let into = |t: &Tile| (t.inverse().apply(o), t.inverse().apply(d));
    let n_tiles = picture.tiles.len();
    if walls {
        let m = &picture.walls;
        let tris = m.body_indices as usize / 3;
        let range = if tile_walls { 0..n_tiles } else { 0..1 };
        for ti in range {
            let (o, d) = into(&picture.tiles[ti]);
            for t in 0..tris {
                let v = |k: usize| m.vertices[m.triangles[3 * t + k] as usize].pos;
                if let Some(dist) = triangle(o, d, v(0), v(1), v(2)) {
                    let point = [0, 1, 2].map(|c| o[c] + dist * d[c]);
                    consider(Hit { kind: HitKind::Wall { face: m.face_of[t] as usize }, t: dist, point, tile: ti });
                }
            }
        }
    }
    for draw in sheets {
        let tiles: Vec<usize> = match draw.tiles {
            TileSel::All => (0..n_tiles).collect(),
            TileSel::One(t) => vec![(t as usize).min(n_tiles - 1)],
        };
        let mesh = &draw.mesh;
        for ti in tiles {
            let (o, d) = into(&picture.tiles[ti]);
            for t in 0..mesh.triangles.len() / 3 {
                let v = |k: usize| mesh.vertices[mesh.triangles[3 * t + k] as usize].pos;
                if let Some(dist) = triangle(o, d, v(0), v(1), v(2)) {
                    let point = [0, 1, 2].map(|c| o[c] + dist * d[c]);
                    if window.contains(point) {
                        consider(Hit { kind: HitKind::Sheet { cell: mesh.cell_of[t] as usize }, t: dist, point, tile: ti });
                    }
                }
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::super::scene3::tests::anf;
    use super::super::scene3::{world, Extent, Sheet};
    use super::*;
    use bowshock_core::Part;
    use std::sync::Arc;

    fn prepared() -> Prepared3 {
        Prepared3::new(anf())
    }

    /// A world-frame ray from the point `from` (solver frame) to the point `at` (solver frame).
    fn ray(from: [f32; 3], at: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        let d = sub(at, from);
        let n = dot(d, d).sqrt();
        (world(from), world([d[0] / n, d[1] / n, d[2] / n]))
    }

    #[test]
    fn a_ray_at_the_cone_hits_its_wall_in_the_tile_it_looks_at() {
        let p = prepared();
        // the nose cone, half way along (x 2.8), seen from the +y side (the solved half) and from the -y side (the mirrored copy)
        let r = 0.5 * 2.8 / 2.84;
        for (side, tile) in [(1.0f32, 0usize), (-1.0, 1)] {
            let (o, d) = ray([2.8, side * 5.0, 0.0], [2.8, side * r, 0.0]);
            let hit = pick(&p, o, d, &[], p.window, true, true).expect("the cone is hit");
            let HitKind::Wall { face } = hit.kind else { panic!("{hit:?}") };
            assert_eq!(p.field.walls[face].part, Part::Nose);
            assert_eq!(hit.tile, tile);
            assert!((hit.t - (5.0 - r)).abs() < 0.05, "{hit:?}");
            // the point comes back in the solved domain's frame: y >= 0
            assert!(hit.point[1] >= -1e-4 && (hit.point[1].hypot(hit.point[2]) - r).abs() < 0.05, "{hit:?}");
        }
        // only the solved half is there without tiling
        let (o, d) = ray([2.8, -5.0, 0.0], [2.8, -r, 0.0]);
        let through = pick(&p, o, d, &[], p.window, true, false).expect("the far side of the half domain is hit from inside");
        assert_eq!(through.tile, 0);
        assert!(through.t > 5.0);
        // a ray past the body hits nothing
        let (o, d) = ray([2.8, 5.0, 3.0], [2.8, 3.0, 3.0]);
        assert_eq!(pick(&p, o, d, &[], p.window, true, true), None);
    }

    #[test]
    fn a_sheet_in_front_of_the_wall_is_hit_first_and_reports_its_cell() {
        let p = prepared();
        let k = p.field.layer_of_theta(0.0);
        let draws = p.sheet_draws(Sheet::Meridian(k), p.window, false);
        // straight at the sheet from its side, at x = 6, 1.2 from the axis: the sheet is hit before anything else
        let theta = p.field.theta_of_layer(k);
        let at_radius = |r: f32| {
            let (s, c) = theta.sin_cos();
            ray([6.0, r * s + 3.0 * c, r * c - 3.0 * s], [6.0, r * s, r * c])
        };
        let (o, d) = at_radius(1.2);
        let hit = pick(&p, o, d, &draws, p.window, true, true).expect("the sheet is hit");
        assert!((hit.t - 3.0).abs() < 0.02, "{hit:?}");
        let HitKind::Sheet { cell } = hit.kind else { panic!("{hit:?}") };
        assert!(p.field.is_fluid(cell));
        let c = (cell % p.field.ni, cell / p.field.ni % p.field.nj, cell / (p.field.ni * p.field.nj));
        assert_eq!(c.2, k, "the cell is in the layer of the sheet");
        // the cell is where the ray went in: its centre is near x = 6 and the sheet's distance from the axis
        let centre = p.field.centre(c.0, c.1, c.2);
        assert!((centre[0] - 6.0).abs() < 0.5 && (centre[1].hypot(centre[2]) - 1.2).abs() < 0.6, "{centre:?}");
        // outside the window the sheet is not there
        let far = Arc::new(p.sheet(Sheet::Meridian(k), p.window_of(Extent::Far)));
        let all = [SheetDraw { mesh: far, tiles: TileSel::One(0) }];
        let (o, d) = at_radius(30.0);
        assert_eq!(pick(&p, o, d, &all, p.window, false, false), None, "beyond the window");
        assert!(pick(&p, o, d, &all, p.window_of(Extent::Far), false, false).is_some());
    }
}
